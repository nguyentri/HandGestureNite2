/*******************************************************************************
*                                                                              *
*   PrimeSense NiTE 2.0 - Hand Viewer Sample                                   *
*   Copyright (C) 2012 PrimeSense Ltd.                                         *
*                                                                              *
*******************************************************************************/

#include <map>
#include "HandSegm.h"
#include "Utilities.h"

extern openni::VideoStream m_depthStream;

void HandSegm::HandSegmInit(	IplImage* BinImagP, IplImage* RgbImgP,
				   nite::HandTracker* handTracker,  nite::HandTrackerFrameRef* handFrame)
{
	//init input
	m_pHandTracker = handTracker; 
	phandFrame	= handFrame;
	pBinImag = BinImagP;
	pRgbImg = RgbImgP;
	//img_t = cvCreateImage(cvSize(160, 160), this->pBinImag->depth, this->pBinImag->nChannels);
}

HandSegm::~HandSegm()
{
	cvReleaseImage(&img_t0);
	cvReleaseImage(&img_t);
}

void HandSegm::ReleaseImg()
{
	cvReleaseImage(&img_t0);
	cvReleaseImage(&img_t);
}

void HandSegm::HandDetection()
{
	const nite::Array<nite::GestureData>& gestures = phandFrame->getGestures();
	
	nite::HandId id;
	
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

	const nite::Array<nite::HandData>& hands= phandFrame->getHands();
	CvPoint2D32f handPoint_f[2] = {cvPoint2D32f(0,0), cvPoint2D32f(0,0)};
	CvPoint cvhandPoint[2] = {cvPoint(0, 0), cvPoint(0, 0)};
	 
	for (int i = 0; i < hands.getSize(); ++i)
	{
		const nite::HandData& hand = hands[i];

		if (!hand.isTracking())// Tracking hand is lost
		{
			printf("Lost hand %d\n", hand.getId());
			id = hand.getId();
			HistoryBuffer<HISBUFFER>* pHistory = g_histories[id];
			g_histories.erase(g_histories.find(id));
			delete pHistory;
			handPointClear(handNum);
			this->handFound[handNum - 1] = false;
			handNum--;
		}
		else
		{
			if (hand.isNew())
			{
				id = hand.getId();
				printf("Found hand %d\n", hand.getId());
				g_histories[handNum] = new HistoryBuffer<HISBUFFER>;
				printf("Hand Num %d\n", handNum);
				handNum++;
			}

			for(int i = 0; i < handNum; i++)
			{
				this->handFound[i] = true;
				//id = hand.getId();
				this->hand3DPoint[i] = hand.getPosition();
				// Add to history
				HistoryBuffer<HISBUFFER>* pHistory = g_histories[i];
				pHistory->AddPoint(hand.getPosition());
				// Draw history
				DrawHistory(m_pHandTracker, i, pHistory);
				//Get coordination of hand point
				m_pHandTracker->convertHandCoordinatesToDepth(hand3DPoint[i].x, hand3DPoint[i].y, hand3DPoint[i].z, (float*)&handPoint_f[i].x, (float*)&handPoint_f[i].y);	
				// Get hand depth point
				openni::CoordinateConverter::convertWorldToDepth(m_depthStream, hand3DPoint[i].x, hand3DPoint[i].y, hand3DPoint[i].z, &handPoint[i].p.x, &handPoint[i].p.y, &handPoint[i].d);
				// Convert hand point to integer
				cvhandPoint[i] = cvPointFrom32f(handPoint_f[handNum-1]);
				cvCircle(this->pRgbImg, cvhandPoint[i], 2, RED, 4, CV_AA, 0);
			}
		}
	}
}


void HandSegm::DrawHistory(nite::HandTracker* pHandTracker, int id, HistoryBuffer<HISBUFFER>* pHistory)
{
	CvPoint2D32f Floatcoordinates = cvPoint2D32f(0,0);

	int Colors[][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}, {1, 1, 1}};
	int colorCount = 3;	
	int hisBufSize = pHistory->GetSize();

	CvPoint* coordinates = new CvPoint[hisBufSize];

	CvScalar color = cvScalar (255*(Colors[id % colorCount][0], 255*Colors[id % colorCount][1], 255*Colors[id % colorCount][2]));
	//color = YELLOW;

	for (int i = 0; i < pHistory->GetSize(); ++i)
	{
		const nite::Point3f& position = pHistory->operator[](i);
		pHandTracker->convertHandCoordinatesToDepth(position.x, position.y, position.z, &Floatcoordinates.x, &Floatcoordinates.y);
		coordinates[i] = cvPointFrom32f(Floatcoordinates);
	}
	cvDrawSetofPoints(this->pRgbImg, coordinates, color, hisBufSize);
	free(coordinates);
}


void HandSegm::HandSegmentation(nite::HandId handID)
{
	// Hand offset 1
	//this->handOffset = 80;
	this->handOffset = 64000/handPoint[handID].d;

	int y= 0 , x = 0;
	openni::DepthPixel depth = 0;

	CvRect rect;

	rect.x = (this->handPoint[handID].p.x >= this->handOffset)? (this->handPoint[handID].p.x - this->handOffset) : 0;
	rect.y = (this->handPoint[handID].p.y >= this->handOffset)? (this->handPoint[handID].p.y - this->handOffset) : 0;
	rect.height = this->handOffset<<1;
	rect.width = this->handOffset<<1;
	//rect.height = 150;
	//rect.width = 150;

	rect.x = (rect.x + rect.width < WIDTH)? rect.x : WIDTH - rect.width;
	rect.y = (rect.y + rect.height < HEIGHT)? rect.y : HEIGHT - rect.height;

	this->RectTop = cvPoint(rect.x, rect.y);

	img_t = cvCreateImage(cvSize(rect.width, rect.height), this->pBinImag->depth, this->pBinImag->nChannels);
	//img_t0 = cvCreateImage(cvSize(rect.width, rect.height), this->pBinImag->depth, this->pBinImag->nChannels);

	uchar* pImg_t =  (uchar*)img_t->imageData;
	//uchar* pImg_t0 =  (uchar*)img_t0->imageData;

	cvSetImageROI(this->pBinImag, rect);
	cvCopy(this->pBinImag, img_t, NULL);
	//cvCopy(this->pBinImag, img_t0, NULL);
	cvResetImageROI(this->pBinImag);


	//IplImage* img_t2 = cvCreateImage(cvSize(W, H), this->pImg->depth, this->pImg->nChannels);
	//cvZero(img_t2);

	int t_depthTh2 = ( handPoint[handID].d + (openni::DepthPixel)(100000/handPoint[handID].d ));
	int t_maxCol = (rect.y + rect.height);
	int t_maxRow = (rect.x + rect.width);
	int t_noiseTh = rect.y + rect.height - 10000/handPoint[handID].d;

	int pixelValue = 0;
	for (y = rect.y; y <  t_maxCol; ++y)
	{
		for (x = rect.x; x <  t_maxRow; ++x, pImg_t++//, pImg_t0++
			)
		{
			pixelValue = *pImg_t;

			depth = (openni::DepthPixel)pixelValue*INV_DEPTH_SCALE_FACTOR;

			if ( (pixelValue != 0) && ( depth < t_depthTh2) )
			{
				if (y < t_noiseTh)
				{	
					*pImg_t = 255;
				//cvSetReal2D(img_t2, y, x, 255);
				}
				else
				{
					*pImg_t = 0;
				}
			}
			else
			{
				*pImg_t = 0;
			}
		}
	}
	//Map to public image
	this->pThImg = img_t;
	//cvShowImage("moment image", this->pMmImg);
}

