//   Copyright (C) 2015  Marek Kowalski (M.Kowalski@ire.pw.edu.pl), Jacek Naruniec (J.Naruniec@ire.pw.edu.pl)
//   License: MIT Software License   See LICENSE.txt for the full license.

//   If you use this software in your research, then please use the following citation:

//    Kowalski, M.; Naruniec, J.; Daniluk, M.: "LiveScan3D: A Fast and Inexpensive 3D Data
//    Acquisition System for Multiple Kinect v2 Sensors". in 3D Vision (3DV), 2015 International Conference on, Lyon, France, 2015

//    @INPROCEEDINGS{Kowalski15,
//        author={Kowalski, M. and Naruniec, J. and Daniluk, M.},
//        booktitle={3D Vision (3DV), 2015 International Conference on},
//        title={LiveScan3D: A Fast and Inexpensive 3D Data Acquisition System for Multiple Kinect v2 Sensors},
//        year={2015},
//    }
#include "KinectCapture.h"
#include <chrono>
#include "opencv2\opencv.hpp"

cv::VideoWriter video("outcpp.avi", CV_FOURCC('M', 'J', 'P', 'G'), 30, cv::Size(1920, 1080));

bool iscapturing;
bool captureLock;

KinectCapture::KinectCapture()
{
	pKinectSensor = NULL;
	pCoordinateMapper = NULL;
	pMultiSourceFrameReader = NULL;
}

KinectCapture::~KinectCapture()
{
	SafeRelease(pKinectSensor);
	SafeRelease(pCoordinateMapper);
	SafeRelease(pMultiSourceFrameReader);
}

bool KinectCapture::Initialize()
{
	HRESULT hr;

	hr = GetDefaultKinectSensor(&pKinectSensor);
	if (FAILED(hr))
	{
		bInitialized = false;
		return bInitialized;
	}

	if (pKinectSensor)
	{
		pKinectSensor->get_CoordinateMapper(&pCoordinateMapper);
		hr = pKinectSensor->Open();

		if (SUCCEEDED(hr))
		{
			pKinectSensor->OpenMultiSourceFrameReader(FrameSourceTypes::FrameSourceTypes_Color | 
				FrameSourceTypes::FrameSourceTypes_Depth | 
				FrameSourceTypes::FrameSourceTypes_Body |
				FrameSourceTypes::FrameSourceTypes_BodyIndex, 
				&pMultiSourceFrameReader);
		}
	}

	bInitialized = SUCCEEDED(hr);

	if (bInitialized)
	{
		std::chrono::time_point<std::chrono::system_clock> start = std::chrono::system_clock::now();
		bool bTemp;
		do
		{
			bTemp = AcquireFrame();
			
			std::chrono::duration<double> elapsedSeconds = std::chrono::system_clock::now() - start;
			if (elapsedSeconds.count() > 5.0)
			{
				bInitialized = false;
				break;
			}

		} while (!bTemp);
	}

	iscapturing = false;
	captureLock = false;

	return bInitialized;
}

bool KinectCapture::AcquireFrame()
{
	if (!bInitialized)
	{
		return false;
	}

	//Multi frame
	IMultiSourceFrame* pMultiFrame = NULL;
	HRESULT hr = pMultiSourceFrameReader->AcquireLatestFrame(&pMultiFrame);

	if (!SUCCEEDED(hr))
	{
		return false;
	}

	GetDepthFrame(pMultiFrame);
	GetColorFrame(pMultiFrame);
	GetBodyFrame(pMultiFrame);
	GetBodyIndexFrame(pMultiFrame);

	SafeRelease(pMultiFrame);

	return true;
}

void KinectCapture::MapDepthFrameToCameraSpace(Point3f *pCameraSpacePoints)
{
	pCoordinateMapper->MapDepthFrameToCameraSpace(nDepthFrameWidth * nDepthFrameHeight, pDepth, nDepthFrameWidth * nDepthFrameHeight, (CameraSpacePoint*)pCameraSpacePoints);
}

void KinectCapture::MapColorFrameToCameraSpace(Point3f *pCameraSpacePoints)
{
	pCoordinateMapper->MapColorFrameToCameraSpace(nDepthFrameWidth * nDepthFrameHeight, pDepth, nColorFrameWidth * nColorFrameHeight, (CameraSpacePoint*)pCameraSpacePoints);
}

void KinectCapture::MapDepthFrameToColorSpace(Point2f *pColorSpacePoints)
{
	pCoordinateMapper->MapDepthFrameToColorSpace(nDepthFrameWidth * nDepthFrameHeight, pDepth, nDepthFrameWidth * nDepthFrameHeight, (ColorSpacePoint*)pColorSpacePoints);
}

void KinectCapture::MapColorFrameToDepthSpace(Point2f *pDepthSpacePoints)
{
	pCoordinateMapper->MapColorFrameToDepthSpace(nDepthFrameWidth * nDepthFrameHeight, pDepth, nColorFrameWidth * nColorFrameHeight, (DepthSpacePoint*)pDepthSpacePoints);;
}

void KinectCapture::GetDepthFrame(IMultiSourceFrame* pMultiFrame)
{
	IDepthFrameReference* pDepthFrameReference = NULL;
	IDepthFrame* pDepthFrame = NULL;
	pMultiFrame->get_DepthFrameReference(&pDepthFrameReference);
	HRESULT hr = pDepthFrameReference->AcquireFrame(&pDepthFrame);

	if (SUCCEEDED(hr))
	{
		if (pDepth == NULL)
		{
			IFrameDescription* pFrameDescription = NULL;
			hr = pDepthFrame->get_FrameDescription(&pFrameDescription);
			pFrameDescription->get_Width(&nDepthFrameWidth);
			pFrameDescription->get_Height(&nDepthFrameHeight);
			pDepth = new UINT16[nDepthFrameHeight * nDepthFrameWidth];
			SafeRelease(pFrameDescription);
		}

		UINT nBufferSize = nDepthFrameHeight * nDepthFrameWidth;
		hr = pDepthFrame->CopyFrameDataToArray(nBufferSize, pDepth);
	}

	SafeRelease(pDepthFrame);
	SafeRelease(pDepthFrameReference);
}

void KinectCapture::GetColorFrame(IMultiSourceFrame* pMultiFrame)
{

	IColorFrameReference* pColorFrameReference = NULL;
	IColorFrame* pColorFrame = NULL;
	pMultiFrame->get_ColorFrameReference(&pColorFrameReference);
	HRESULT hr = pColorFrameReference->AcquireFrame(&pColorFrame);

	cv::Mat colorMat(1080, 1920, CV_8UC3);

	if (SUCCEEDED(hr))
	{
		if (pColorRGBX == NULL)
		{
			IFrameDescription* pFrameDescription = NULL;
			hr = pColorFrame->get_FrameDescription(&pFrameDescription);
			hr = pFrameDescription->get_Width(&nColorFrameWidth);
			hr = pFrameDescription->get_Height(&nColorFrameHeight);
			pColorRGBX = new RGB[nColorFrameWidth * nColorFrameHeight];
			SafeRelease(pFrameDescription);
		}

		UINT nBufferSize = nColorFrameWidth * nColorFrameHeight * sizeof(RGB);
		hr = pColorFrame->CopyConvertedFrameDataToArray(nBufferSize, reinterpret_cast<BYTE*>(pColorRGBX), ColorImageFormat_Bgra);

		 //Following code taken from: https://gist.github.com/UnaNancyOwen/bff6c3342839592cd51e
		
			
			UINT bufferSize = 0;
			BYTE* pBuffer = nullptr;
			hr = pColorFrame->AccessRawUnderlyingBuffer(&bufferSize, &pBuffer); // YUY2
			if (SUCCEEDED(hr)) {
				cv::Mat bufferMat(1080, 1920, CV_8UC2, pBuffer);
				cv::cvtColor(bufferMat, colorMat, CV_YUV2BGR_YUYV);
				video.write(colorMat);
			}

	
		
	}

	SafeRelease(pColorFrame);
	SafeRelease(pColorFrameReference);
}

void KinectCapture::GetBodyFrame(IMultiSourceFrame* pMultiFrame)
{
	IBodyFrameReference* pBodyFrameReference = NULL;
	IBodyFrame* pBodyFrame = NULL;
	pMultiFrame->get_BodyFrameReference(&pBodyFrameReference);
	HRESULT hr = pBodyFrameReference->AcquireFrame(&pBodyFrame);


	if (SUCCEEDED(hr))
	{
		IBody* bodies[BODY_COUNT] = { NULL };
		pBodyFrame->GetAndRefreshBodyData(BODY_COUNT, bodies);

		vBodies = std::vector<Body>(BODY_COUNT);
		for (int i = 0; i < BODY_COUNT; i++)
		{		
			if (bodies[i])
			{
				Joint joints[JointType_Count];
				BOOLEAN isTracked;

				bodies[i]->get_IsTracked(&isTracked);
				bodies[i]->GetJoints(JointType_Count, joints);

				vBodies[i].vJoints.assign(joints, joints + JointType_Count);

				if (isTracked == TRUE)
					vBodies[i].bTracked = true;
				else
					vBodies[i].bTracked = false;

				vBodies[i].vJointsInColorSpace.resize(JointType_Count);

				for (int j = 0; j < JointType_Count; j++)
				{
					ColorSpacePoint tempPoint;
					pCoordinateMapper->MapCameraPointToColorSpace(joints[j].Position, &tempPoint);
					vBodies[i].vJointsInColorSpace[j].X = tempPoint.X;
					vBodies[i].vJointsInColorSpace[j].Y = tempPoint.Y;
				}
			}
		}
	}

	SafeRelease(pBodyFrame);
	SafeRelease(pBodyFrameReference);
}

void KinectCapture::GetBodyIndexFrame(IMultiSourceFrame* pMultiFrame)
{
	IBodyIndexFrameReference* pBodyIndexFrameReference = NULL;
	IBodyIndexFrame* pBodyIndexFrame = NULL;
	pMultiFrame->get_BodyIndexFrameReference(&pBodyIndexFrameReference);
	HRESULT hr = pBodyIndexFrameReference->AcquireFrame(&pBodyIndexFrame);


	if (SUCCEEDED(hr))
	{
		if (pBodyIndex == NULL)
		{
			pBodyIndex = new BYTE[nDepthFrameHeight * nDepthFrameWidth];
		}

		UINT nBufferSize = nDepthFrameHeight * nDepthFrameWidth;
		hr = pBodyIndexFrame->CopyFrameDataToArray(nBufferSize, pBodyIndex);
	}

	SafeRelease(pBodyIndexFrame);
	SafeRelease(pBodyIndexFrameReference);
}

void KinectCapture::StopColorTextureRecording() 
{
	video.release();
}