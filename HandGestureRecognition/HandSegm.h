/*******************************************************************************
*                                                                              *
*   PrimeSense NiTE 2.0 - Hand Viewer Sample                                   *
*   Copyright (C) 2012 PrimeSense Ltd.                                         *
*                                                                              *
*******************************************************************************/

#ifndef _HAND_SEG_H_
#define _HAND_SEG_H_

using namespace std;

#include "OpenNI.h"
#include "NiTE.h"
#include "HandViewer.h"
#include "HistoryBuffer.h"
#include "Utilities.h"

#define HISBUFFER  20

typedef struct cvDPoint
{
	CvPoint p;
	openni::DepthPixel d;
}cvDPoint;

typedef struct cvImgPoint
{
	CvPoint p;
	uint8_t val;
}cvImgPoint;


//	openni::VideoStream depth, color;

class HandSegm
{
public:
	void HandSegm::HandSegmInit(	IplImage* BinImagP, IplImage* RgbImgP,
				   nite::HandTracker* handTracker,  nite::HandTrackerFrameRef* handFrame);
	HandSegm(){	
		//Init ouput
		handNum = 0;
		pThImg = NULL;
		pMmImg = NULL;
		//temporate pointer
		img_t0 = NULL;
		img_t = NULL;
		handFound[0] = false;
		handFound[1] = false;
		};
	~HandSegm();

	void HandDetection();

	void HandSegm::HandSegmentation(nite::HandId handID);

	uint8_t GetNumHand(){return handNum;};

	void ReleaseImg();

	IplImage* pThImg;
	IplImage* pMmImg;
	bool handFound[2];	
	cvDPoint handPoint[2];

	CvPoint RectTop;

protected:
	void DrawHistory(nite::HandTracker* pHandTracker, int id, HistoryBuffer<HISBUFFER>* pHistory);

private:
	HandSegm(const HandSegm&);
	HandSegm& operator=(HandSegm&);
	void handPointClear(nite::HandId handId){
			hand3DPoint[handId].x = 0; hand3DPoint[handId].y = 0; hand3DPoint[handId].z = 0;
		}; 	
	/*
	*Private members
	*/
	CvPoint* cvhandPoint[2];
	int handOffset;

	/*NITE private members */
	/*input data */
	nite::HandTracker* m_pHandTracker;
	nite::HandTrackerFrameRef* phandFrame;

	NitePoint3f hand3DPoint[2];
	/*hand point with depthData */
	IplImage* img_t;
	IplImage* img_t0;
	IplImage* pBinImag;
	IplImage* pRgbImg;	
	
	std::map<int, HistoryBuffer<HISBUFFER> *> g_histories;
	uint8_t handNum;
};


#endif // _HAND_SEG_H_
