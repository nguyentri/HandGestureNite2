/*******************************************************************************
*                                                                              *
*   PrimeSense NiTE 2.0 - Hand Viewer Sample                                   *
*   Copyright (C) 2012 PrimeSense Ltd.                                         *
*                                                                              *
*******************************************************************************/

#include <map>
#include "HandViewer.h"
#include <Utilities.h>

using namespace std;

HandViewer* HandViewer::ms_self = NULL;

std::map<int, HistoryBuffer<HISBUFFER> *> g_histories;

bool g_drawDepth = true;
bool g_smoothing = false;
bool g_drawFrameId = false;

int g_nXRes = 0, g_nYRes = 0;

openni::VideoStream m_depthStream;



void HandViewer::cvKeyboard(unsigned char key, int x, int y)
{
	HandViewer::ms_self->OnKey(key, x, y);
}

HandViewer::HandViewer(const char* strSampleName) : handFound(false), handOffset(0)
{
	ms_self = this;
	strncpy_s(m_strSampleName, strSampleName, ONI_MAX_STR);
	m_pHandTracker = new nite::HandTracker;
}
HandViewer::~HandViewer()
{
	Finalize();
	ms_self = NULL;
}

void HandViewer::Finalize()
{
	delete m_pHandTracker;
	nite::NiTE::shutdown();
	openni::OpenNI::shutdown();

	/*Release structure images */
	cvReleaseImage(&pImg);
	cvReleaseImage(&pThImg);
}

openni::Status HandViewer::Init(int argc, char **argv)
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


	rc = m_depthStream.create(m_device, openni::SENSOR_DEPTH);
	if (rc == openni::STATUS_OK)
	{
		rc = m_depthStream.start();
		if (rc != openni::STATUS_OK)
		{
			printf("SimpleViewer: Couldn't start depth stream:\n%s\n", openni::OpenNI::getExtendedError());
			m_depthStream.destroy();
		}
	}
	else
	{
		printf("SimpleViewer: Couldn't find depth stream:\n%s\n", openni::OpenNI::getExtendedError());
	}


	m_pHandTracker->startGestureDetection(nite::GESTURE_WAVE);
	m_pHandTracker->startGestureDetection(nite::GESTURE_CLICK);
	m_pHandTracker->startGestureDetection(nite::GESTURE_HAND_RAISE);

	return InitOpenCV(argc, argv);

}

openni::Status HandViewer::Run()	//Does not return
{
	int key;
	this->HandSegmentation();
	this->HandDisplay();
	key = cvWaitKey(1);
	HandViewer::cvKeyboard(key,NULL,NULL);
	return openni::STATUS_OK;
}


void HandViewer::HandSegmentation()
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
			HistoryBuffer<HISBUFFER>* pHistory = g_histories[id];
			g_histories.erase(g_histories.find(id));
			delete pHistory;
			handPointClear();
		}
		else
		{
			if (hand.isNew())
			{
				printf("Found hand %d\n", hand.getId());
				nite::HandId id = hand.getId();
				g_histories[hand.getId()] = new HistoryBuffer<HISBUFFER>;
				handPoint = cvPoint2D32f(0,0);
				cvhandPoint = cvPoint(0, 0);
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
		
			this->getHandThreshold();

			cvCircle(this->pRgbImg, cvhandPoint, 2, RED, 4, CV_AA, 0);
		}
	}

}


int Colors[][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}, {1, 1, 1}};
int colorCount = 3;

void HandViewer::DrawHistory(nite::HandTracker* pHandTracker, int id, HistoryBuffer<HISBUFFER>* pHistory)
{
	CvPoint2D32f Floatcoordinates = cvPoint2D32f(0,0);

	int hisBufSize = pHistory->GetSize();

	CvPoint* coordinates = new CvPoint[hisBufSize];

	CvScalar color = cvScalar (255*(Colors[id % colorCount][0], 255*Colors[id % colorCount][1], 255*Colors[id % colorCount][2]));
	color = YELLOW;

	for (int i = 0; i < pHistory->GetSize(); ++i)
	{
		const nite::Point3f& position = pHistory->operator[](i);
		pHandTracker->convertHandCoordinatesToDepth(position.x, position.y, position.z, &Floatcoordinates.x, &Floatcoordinates.y);
		coordinates[i] = cvPointFrom32f(Floatcoordinates);
	}

	cvDrawSetofPoints(this->pRgbImg, coordinates, color, hisBufSize);

}


void HandViewer:: HandDisplay()
{	
	cvShowImage("DepthImage", this->pRgbImg);	
	cvShowImage("ThresholdImage", this->pThImg);
}

nite::Status HandViewer::getHandThreshold()
{
	// Get hand depth point
	openni::CoordinateConverter::convertWorldToDepth(m_depthStream, hand3DPoint.x, hand3DPoint.y, hand3DPoint.z, &handDepthPoint.p.x, &handDepthPoint.p.y, &handDepthPoint.d);

	// Hand offset 
	this->handOffset = 60000/handDepthPoint.d;

	int y= 0 , x = 0;
	CvRect rect;

	rect.x = (this->handDepthPoint.p.x >= this->handOffset)? (this->handDepthPoint.p.x - this->handOffset) : 0;
	rect.y = (this->handDepthPoint.p.y >= this->handOffset)? (this->handDepthPoint.p.y - this->handOffset) : 0;
	rect.height = handOffset<<1;
	rect.width = handOffset<<1;

	rect.x = (rect.x + rect.width < W)? rect.x : W - rect.width;
	rect.y = (rect.y + rect.height < H)? rect.y : H - rect.height;

	this->pThImg = cvCreateImage(cvSize(rect.width, rect.height), this->pImg->depth, this->pImg->nChannels);
	uchar* pImg_t =  (uchar*)this->pThImg->imageData;
	cvSetImageROI(this->pImg, rect);
	cvCopy(this->pImg, this->pThImg, NULL);
	cvResetImageROI(this->pImg);

	int pixelValue = 0;
	for (y = rect.y; y <  (rect.y + rect.height); ++y)
	{
		for (x = rect.x; x <  (rect.x + rect.width); ++x, pImg_t++)
		{
			pixelValue = *pImg_t;
			if (*pImg_t != 0 && (*pImg_t/DEPTH_SCALE_FACTOR) < (handDepthPoint.d + 100000/handDepthPoint.d))
			{	
				*pImg_t = 255;
			}
			else
			{
				*pImg_t = 0;
			}
		}
	}
	return nite::STATUS_OK;
}


void HandViewer::OnKey(unsigned char key, int /*x*/, int /*y*/)
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
			m_pHandTracker->setSmoothingFactor((float)0.1);
			g_smoothing = TRUE;
		}
		break;
	case 'f':
		// Draw frame ID
		g_drawFrameId = !g_drawFrameId;
		break;
	}

}

openni::Status HandViewer::InitOpenCV(int argc, char **argv)
{
	// Initialize HandSegmentation windows
	cvNamedWindow("DepthImage", CV_WINDOW_NORMAL);
	cvNamedWindow("ThresholdImage", CV_WINDOW_NORMAL);
	cvMoveWindow("DepthImage", 20, 50);
	cvMoveWindow("ThresholdImage", 800, 50);

	// Initialize pointers that point to images
	this->pImg = cvCreateImage(cvSize(W, H), IPL_DEPTH_8U, 1);
	this->pThImg = cvCreateImage(cvSize(W, H), IPL_DEPTH_8U, 1);
	this->pRgbImg = cvCreateImage(cvSize(W, H), IPL_DEPTH_8U, 3);

	return openni::STATUS_OK;
}
