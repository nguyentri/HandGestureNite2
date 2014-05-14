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


#define MAX_DEPTH 4000
#define W		  640
#define H	      480


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



class HandGesture
{
public:
	HandGesture(const char* strSampleName);
	virtual ~HandGesture();

	virtual openni::Status Init(int argc, char **argv);
	virtual openni::Status Run();	//Does not return

protected:
	virtual void Display();
	virtual void DisplayPostDraw(){};	// Overload to draw over the screen image

	virtual void OnKey(unsigned char key, int x, int y);

	virtual openni::Status InitOpenCV(int argc, char **argv);

	void Finalize();

private:
	HandGesture(const HandGesture&);
	HandGesture& operator=(HandGesture&);

	static HandGesture* ms_self;
	NitePoint3f hand3DPoint;
	static const CvPoint& cvhandPoint;

	static void cvDisplay();
	static void cvKeyboard(unsigned char key, int x, int y);

	void getHandThreshold(openni::VideoFrameRef depthFrame);

	int				m_pDepthHist[MAX_DEPTH];
	char			m_strSampleName[ONI_MAX_STR];

	openni::Device		m_device;
	nite::HandTracker* m_pHandTracker;


	cvDepthPoint handPoint_l;
	IplImage* pRgbImg;
	IplImage* pImg;
 	IplImage* pThImg;
};


#endif // _NITE_HAND_VIEWER_H_
