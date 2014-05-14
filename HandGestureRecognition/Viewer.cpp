/*******************************************************************************
*                                                                              *
*   PrimeSense NiTE 2.0 - Hand Viewer Sample                                   *
*   Copyright (C) 2012 PrimeSense Ltd.                                         *
*                                                                              *
*******************************************************************************/

#include <map>
#include "Viewer.h"
#include <NiteSampleUtilities.h>

using namespace std;


#define HISBUFFER  1

HandGesture* HandGesture::ms_self = NULL;

bool g_drawDepth = true;
bool g_smoothing = false;
bool g_drawFrameId = false;

int g_nXRes = 0, g_nYRes = 0;


void HandGesture::cvDisplay()
{
	HandGesture::ms_self->Display();
}

void HandGesture::cvKeyboard(unsigned char key, int x, int y)
{
	HandGesture::ms_self->OnKey(key, x, y);
}

HandGesture::HandGesture(const char* strSampleName)
{
	ms_self = this;
	strncpy(m_strSampleName, strSampleName, ONI_MAX_STR);
	m_pHandTracker = new nite::HandTracker;
}
HandGesture::~HandGesture()
{
	Finalize();
	ms_self = NULL;
}

void HandGesture::Finalize()
{
	delete m_pHandTracker;
	nite::NiTE::shutdown();
	openni::OpenNI::shutdown();

	/*Release structure images */
	cvReleaseImage(&pImg);
	cvReleaseImage(&pThImg);
}

openni::Status HandGesture::Init(int argc, char **argv)
{

	openni::OpenNI::initialize();

	const char* deviceUri = openni::ANY_DEVICE;
	for (int i = 1; i < argc-1; ++i)
	{
		if (strcmp(argv[i], "-device") == 0)
		{
			deviceUri = argv[i+1];
			break;
		}
	}

	openni::Status rc = m_device.open(deviceUri);
	if (rc != openni::STATUS_OK)
	{
		printf("Open Device failed:\n%s\n", openni::OpenNI::getExtendedError());
		return rc;
	}

	nite::NiTE::initialize();

	if (m_pHandTracker->create(&m_device) != nite::STATUS_OK)
	{
		return openni::STATUS_ERROR;
	}

	m_pHandTracker->startGestureDetection(nite::GESTURE_WAVE);
	m_pHandTracker->startGestureDetection(nite::GESTURE_CLICK);
	m_pHandTracker->startGestureDetection(nite::GESTURE_HAND_RAISE);

	return InitOpenCV(argc, argv);

}
openni::Status HandGesture::Run()	//Does not return
{
	int key;
	this->Display();
	key = cvWaitKey(1);
	HandGesture::cvKeyboard(key,NULL,NULL);
	return openni::STATUS_OK;
}


void HandGesture::Display()
{
	nite::HandTrackerFrameRef handFrame;
	openni::VideoFrameRef depthFrame;

	nite::Status rc = m_pHandTracker->readFrame(&handFrame);

	IplImage* pDepthImg = cvCreateImage(cvSize(W,H), IPL_DEPTH_16U, 1);

	if (rc != nite::STATUS_OK)
	{
		printf("GetNextData failed\n");
		return;
	}

	depthFrame = handFrame.getDepthFrame();

	if (depthFrame.isValid())
	{
//		calculateHistogram(m_pDepthHist, MAX_DEPTH, depthFrame);
//		getHandThreshold(depthFrame);
		/*Copy depth data to image data*/
		memcpy(pDepthImg->imageData, depthFrame.getData(), W*H*2);

		cvCvtScale(pDepthImg, this->pImg, DEPTH_SCALE_FACTOR);

		cvConvertImage(this->pImg, this->pRgbImg, CV_GRAY2RGB);
	}

	const nite::Array<nite::GestureData>& gestures = handFrame.getGestures();
	for (int i = 0; i < gestures.getSize(); ++i)
	{
		if (gestures[i].isComplete())
		{
			nite::HandId newId;
			m_pHandTracker->startHandTracking(gestures[i].getCurrentPosition(), &newId);
		}
	}

	const nite::Array<nite::HandData>& hands= handFrame.getHands();
	CvPoint2D32f handPoint = cvPoint2D32f(0,0);
	CvPoint cvhandPoint = cvPoint(0, 0);
	for (int i = 0; i < hands.getSize(); ++i)
	{
		const nite::HandData& hand = hands[i];

		if (!hand.isTracking())// Tracking hand is lost
		{
			printf("Lost hand %d\n", hand.getId());
			nite::HandId id = hand.getId();
			//HistoryBuffer<HISBUFFER>* pHistory = g_histories[id];
			//g_histories.erase(g_histories.find(id));
			//delete pHistory;
			handPoint = cvPoint2D32f(0,0);
			cvhandPoint = cvPoint(0, 0);
		}
		else
		{
			if (hand.isNew())
			{
				printf("Found hand %d\n", hand.getId());
				nite::HandId id = hand.getId();
				//g_histories[hand.getId()] = new HistoryBuffer<HISBUFFER>;
				//handPoint = cvPoint2D32f(0,0);
				//cvhandPoint = cvPoint(0, 0);
				hand3DPoint = hand.getPosition();
			}

			// Add to history
			//HistoryBuffer<HISBUFFER>* pHistory = g_histories[hand.getId()];
			//pHistory->AddPoint(hand.getPosition());
			// Draw history

			printf("Gesture %d at (%f,%f,%f)\n", hand.getId(), hand3DPoint.x, hand3DPoint.y, hand3DPoint.z);

			m_pHandTracker->convertHandCoordinatesToDepth(hand3DPoint.x, hand3DPoint.y, hand3DPoint.z, (float*)&handPoint.x, (float*)&handPoint.y);
			//openni::CoordinateConverter::convertWorldToDepth(depthFrame, hand3DPoint.x, hand3DPoint.y, hand3DPoint.z, &handPoint_l.p.x, &handPoint_l.p.y, &handPoint_l.d);

			cvhandPoint = cvPointFrom32f(handPoint);

			handPoint_l.d = pDepthImg->imageData[(cvhandPoint.x*cvhandPoint.y - 1)<<2];

			printf("Gesture %d at (%d,%d,%d)\n", hand.getId(), cvhandPoint.x, cvhandPoint.y, handPoint_l.d);
			

			cvCircle(this->pRgbImg, cvhandPoint, 2, RED, 4, CV_AA, 0);
		}
	}
	
	cvShowImage("DepthImage", this->pRgbImg);
}





void HandGesture::getHandThreshold(openni::VideoFrameRef depthFrame)
{
	// Get depth map
	const openni::DepthPixel* pDepth = (const openni::DepthPixel*)depthFrame.getData();
	char* pImg_t =  this->pImg->imageData;

	int nHistValue = 0;
	for (int y = 0; y < H; ++y)
	{
		for (int x = 0; x < W; ++x, pImg_t++, pDepth++)
		{
			if (*pDepth !=0)
			{	
				int nHistValue = (int)m_pDepthHist[*pDepth];
				*pImg_t = nHistValue;
//				*pImg_t++ = nHistValue;
//				*pImg_t++ = nHistValue;
			}
			else
			{
				*pImg_t = 0;
//				*pImg_t++ = 0;	
//				*pImg_t++ = 0;
			}
		}
	}
}


void HandGesture::OnKey(unsigned char key, int /*x*/, int /*y*/)
{
	switch (key)
	{
	case 27:
		Finalize();
		exit (1);
	case 'd':
		g_drawDepth = !g_drawDepth;
		break;
	case 's':
		if (g_smoothing)
		{
			// Turn off smoothing
			m_pHandTracker->setSmoothingFactor(0);
			g_smoothing = FALSE;
		}
		else
		{
			m_pHandTracker->setSmoothingFactor(0.1);
			g_smoothing = TRUE;
		}
		break;
	case 'f':
		// Draw frame ID
		g_drawFrameId = !g_drawFrameId;
		break;
	}

}

openni::Status HandGesture::InitOpenCV(int argc, char **argv)
{
	// Initialize display windows
	cvNamedWindow("DepthImage", CV_WINDOW_NORMAL);
//	cvNamedWindow("ThresholdImage", CV_WINDOW_NORMAL);
//	cvMoveWindow("DepthImage", 20, 50);
//	cvMoveWindow("ThresholdImage", 800, 50);

	// Initialize pointers that point to images
	this->pImg = cvCreateImage(cvSize(W, H), IPL_DEPTH_8U, 1);
	this->pThImg = cvCreateImage(cvSize(W, H), IPL_DEPTH_8U, 1);
	this->pRgbImg = cvCreateImage(cvSize(W, H), IPL_DEPTH_8U, 3);

	return openni::STATUS_OK;
}
