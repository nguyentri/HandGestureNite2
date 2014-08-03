/*******************************************************************************
*                                                                              *
*   PrimeSense NiTE 2.0 - Hand Viewer Sample                                   *
*   Copyright (C) 2012 PrimeSense Ltd.                                         *
*                                                                              *
*******************************************************************************/

#include <map>
#include "HandViewer.h"
#include "Utilities.h"
#include "GestureRecognition.h"

using namespace std;

openni::VideoStream m_depthStream;

/* std::map<int, HistoryBuffer<HISBUFFER> *> g_histories;

bool g_drawDepth = true;
bool g_smoothing = false;
bool g_drawFrameId = false;

int g_nXRes = 0, g_nYRes = 0;
 */



//Contructor
HandViewer::HandViewer(const char* strSampleName, const char* strMWdName, const char* strThrWdName) //: handFound(false), handOffset(0)
{
	ms_self = this;
	strncpy_s(m_strSampleName, strSampleName, MAX_STR_NAME);
	strncpy_s(m_MainImgName, strMWdName, MAX_STR_NAME);
	strncpy_s(m_ThreImgName, strThrWdName, MAX_STR_NAME);
	this->ppDisplayImg = &this->pDepthImg;
	this->m_pHandTracker = new nite::HandTracker;
}

//Decontructor
HandViewer::~HandViewer()
{
	delete m_pHandTracker;
	ms_self = NULL;
	nite::NiTE::shutdown();
	openni::OpenNI::shutdown();
}

//
openni::Status HandViewer::Init(int argc, char **argv)
{
	if((InitOpenNI() != openni::STATUS_OK) || (InitOpenCV(argc, argv) != openni::STATUS_OK))
	{
		return openni::STATUS_ERROR;		
	}
	else
	{
		return openni::STATUS_OK;
	}
}

void HandViewer::KeyBoard(unsigned char key, int x, int y)
{
	HandViewer::ms_self->OnKey(key, x, y);
}

/*
* OpenNI/NITE inits. 
* This method shall intialize depth stream, color stream and call starting tracking hand.
*/
openni::Status HandViewer::InitOpenNI(void)
{
	openni::OpenNI::initialize();

	/*Open Kinect sensor */
	openni::Status rc = m_device.open(openni::ANY_DEVICE);
	if (rc != openni::STATUS_OK)
	{	
		/*Open Device failed */
		printf("Open Device failed:\n%s\n", openni::OpenNI::getExtendedError());
		return rc;
	}

	/*Open depth video stream */
	rc = m_depthStream.create(m_device, openni::SENSOR_DEPTH);
	if (rc == openni::STATUS_OK)
	{
		rc = m_depthStream.start();
		if (rc != openni::STATUS_OK)
		{
			printf("HandGesture: Couldn't start depth stream:\n%s\n", openni::OpenNI::getExtendedError());
			m_depthStream.destroy();
			return rc; 
		}
	}
	else
	{
		printf("HandGesture: Couldn't find depth stream:\n%s\n", openni::OpenNI::getExtendedError());
	}

	/*Open color video stream */
	rc = m_colorStream.create(m_device, openni::SENSOR_COLOR);
	if (rc == openni::STATUS_OK)
	{
		rc = m_colorStream.start();
		if (rc != openni::STATUS_OK)
		{
			printf("HandGesture: Couldn't start color stream:\n%s\n", openni::OpenNI::getExtendedError());
			m_colorStream.destroy();
			return rc;
		}
	}	
	else
	{
		printf("HandGesture: Couldn't find color stream:\n%s\n", openni::OpenNI::getExtendedError());
	}

	
	/*NITE int */	
	nite::NiTE::initialize();

	/*Create hand tracking with m_device */
	if (m_pHandTracker->create(&m_device) != nite::STATUS_OK) {
		rc = openni::STATUS_ERROR; 
	}

	m_pHandTracker->startGestureDetection(nite::GESTURE_WAVE);

	return rc; 
}

openni::Status HandViewer::InitOpenCV(int argc, char **argv)
{
	// Initialize HandSegmentation windows
	cvNamedWindow(m_MainImgName, CV_WINDOW_AUTOSIZE);
	cvNamedWindow(m_ThreImgName, CV_WINDOW_NORMAL);
	cvMoveWindow(m_MainImgName, 20, 50);
	cvMoveWindow(m_ThreImgName, 800, 50);

	// Initialize pointers that point to images
	this->pBiDepthImg = cvCreateImage(cvSize(W, H), IPL_DEPTH_8U, 1);
	this->p2BiDepthImg = cvCreateImage(cvSize(W, H), IPL_DEPTH_16U, 1);
	this->pDepthImg = cvCreateImage(cvSize(W, H), IPL_DEPTH_8U, 3);
	this->pColorImg = cvCreateImage(cvSize(W, H), IPL_DEPTH_8U, 3);
	
	return openni::STATUS_OK;
}


//KeyBoard ReadImages
void HandViewer::OnKey(unsigned char key, int /*x*/, int /*y*/)
{
	switch (key)
	{
	case 27:
		HandViewer::~HandViewer();
		exit (1);
	case 'd':
		this->ppDisplayImg = &this->pDepthImg;
		break;
	case 'c':
		this->ppDisplayImg = &this->pColorImg;
		break;
	case 'f':
		// 
		break;
	default:
		break;
	}
	this->pDisplayImg = *this->ppDisplayImg;
}

//ReadImages Images
void HandViewer::ReadImages()
{
	openni::VideoFrameRef depthFrame;
	openni::VideoFrameRef colorFrame;
	
	// Read depth frame
	nite::Status rc = m_pHandTracker->readFrame(&handFrame);
	if (rc != nite::STATUS_OK)
	{
		printf("GetNextData failed\n");
		return;
	}
	depthFrame = handFrame.getDepthFrame();
	// Map depth frame to OpenCV image structure
	if (depthFrame.isValid())
	{
		memcpy(p2BiDepthImg->imageData, depthFrame.getData(), W*H*2);
		/*Convert depth data to binary image */
		cvCvtScale(p2BiDepthImg, this->pBiDepthImg, DEPTH_SCALE_FACTOR);
		/*Convert binary depth image to RGB Depth Image */
		cvConvertImage(this->pBiDepthImg, this->pDepthImg, CV_GRAY2RGB);
	}
	else
	{
		/* depthFrame is invalid */
		printf("HandGesture: Couldn't get depth frame:\n%s\n", openni::OpenNI::getExtendedError());
	}

	//Read Colour frame
	if (m_colorStream.readFrame(&colorFrame) == openni::STATUS_OK)
	{
		/*Copy to Opencv image struture  */
		memcpy(pColorImg->imageData, colorFrame.getData(), W*H*3);
		cvConvertImage(this->pColorImg, this->pColorImg, CV_BGR2RGBA);
	}
	else
	{
		/* color frame is invalid */
		printf("HandGesture: Couldn't get color frame:\n%s\n", openni::OpenNI::getExtendedError());
	}
}


void HandViewer::DisPlayImg(void)
{
	cvShowImage(this->m_MainImgName, pDisplayImg);
}
void HandViewer::DisPlayImg(IplImage* pthrhImg)
{
	if(pthrhImg != NULL)
	{
		cvShowImage(this->m_MainImgName, pDisplayImg);
	}
	if(pthrhImg != NULL)
	{
		cvShowImage(this->m_ThreImgName, pthrhImg);
	}
}


#if 0
void HandViewer::GetHandPoint()
{
	const nite::Array<nite::GestureData>& gestures = handFrame.getGestures();
	for (int i = 0; i < gestures.getSize(); ++i)
	{
		if (gestures[i].isComplete())
		{
			const nite::Point3f& position = gestures[i].getCurrentPosition();
			printf("Gesture %d at (%f,%f,%f)\n", gestures[i].getType(), position.x, position.y, position.z);

			nite::HandId newId;
			m_pHandTracker->startHandTracking(gestures[i].getCurrentPosition(), &newId);
		}
	}

	const nite::Array<nite::HandData>& = handFrame.getHands();
	CvPoint2D32f handPoint = cvPoint2D32f(0,0);
	CvPoint cvhandPoint = cvPoint(0, 0);
	for (int i = 0; i < hands.getSize(); ++i)
	{
		const nite::HandData& hand = hands[i];

		if (!hand.isTracking())// Tracking hand is lost
		{
			printf("Lost hand %d\n", hand.getId());
			nite::HandId id = hand.getId();
			HistoryBuffer<HISBUFFER>* pHistory = g_histories[id];
			g_histories.erase(g_histories.find(id));
			delete pHistory;
			handPointClear();

			this->handFound = false;
		}
		else
		{
			this->handFound = true;

			if (hand.isNew())
			{
				printf("Found hand %d\n", hand.getId());
				nite::HandId id = hand.getId();
				g_histories[hand.getId()] = new HistoryBuffer<HISBUFFER>;
			}
			this->hand3DPoint = hand.getPosition();
			// Add to history
			HistoryBuffer<HISBUFFER>* pHistory = g_histories[hand.getId()];
			pHistory->AddPoint(hand.getPosition());
			// Draw history
			DrawHistory(m_pHandTracker, hand.getId(), pHistory);

			//Get coordiation of hand point
			m_pHandTracker->convertHandCoordinatesToDepth(hand3DPoint.x, hand3DPoint.y, hand3DPoint.z, (float*)&handPoint.x, (float*)&handPoint.y);
			// Convert hand point to int
			cvhandPoint = cvPointFrom32f(handPoint);		
			//Get hand
			this->pThImg = this->getHandThreshold();

			cvCircle(this->pRgbImg, cvhandPoint, 2, RED, 4, CV_AA, 0);
		}
	}
}
#endif

