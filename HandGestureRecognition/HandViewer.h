/*******************************************************************************
*                                                                              *
*   PrimeSense NiTE 2.0 - Hand Viewer Sample                                   *
*   Copyright (C) 2012 PrimeSense Ltd.                                         *
*                                                                              *
*******************************************************************************/

#ifndef _NITE_HAND_VIEWER_H_
#define _NITE_HAND_VIEWER_H_


#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/highgui/highgui_c.h>

#include "OpenNI.h"
#include "NiTE.h"
#include "HistoryBuffer.h"


#define MAX_DEPTH 4000
#define W		  640
#define H	      480



#define HISBUFFER  20
extern std::map<int, HistoryBuffer<HISBUFFER> *> g_histories;


typedef struct cvDepthPoint
{
	CvPoint p;
	uint16_t d;
}cvDepthPoint;

typedef struct cvImgPoint
{
	CvPoint p;
	uint8_t val;
}cvImgPoint;


//	openni::VideoStream depth, color;

class HandViewer
{
public:
	HandViewer(const char* strSampleName);
	virtual ~HandViewer();

	virtual openni::Status Init(int argc, char **argv);
	virtual openni::Status Run();	//Does not return

protected:
	virtual void HandSegmentation();

	virtual void HandViewerDisplay();

	virtual void OnKey(unsigned char key, int x, int y);

	virtual openni::Status InitOpenCV(int argc, char **argv);

	void Finalize();

	void DrawHistory(nite::HandTracker* pHandTracker, int id, HistoryBuffer<HISBUFFER>* pHistory);

private:
	HandViewer(const HandViewer&);
	HandViewer& operator=(HandViewer&);
	void handPointClear(){hand3DPoint.x  = 0; hand3DPoint.y = 0; hand3DPoint.z = 0;}; 

	static HandViewer* ms_self;
	static const CvPoint& cvhandPoint;
	static void cvKeyboard(unsigned char key, int x, int y);

	IplImage* HandViewer::getHandThreshold();

	NitePoint3f hand3DPoint;
	int				m_pDepthHist[MAX_DEPTH];
	char			m_strSampleName[ONI_MAX_STR];

	openni::Device		m_device;
	openni::VideoStream* m_pstream;
	nite::HandTracker* m_pHandTracker;

	int handOffset;
	bool handFound;

	cvDepthPoint handDepthPoint;
	IplImage* pRgbImg;
	IplImage* pImg;
 	IplImage* pThImg;

	CvVideoWriter	*writer;	/* File recording handle */
};


#endif // _NITE_HAND_VIEWER_H_
