/*******************************************************************************
*                                                                              *
*   PrimeSense NiTE 2.0 - Common Utilities for Samples                         *
*   Copyright (C) 2012 PrimeSense Ltd.                                         *
*                                                                              *
*******************************************************************************/

#ifndef _UTILITIES_H_
#define _UTILITIES_H_

#include <stdio.h>
#include <OpenNI.h>

/*Open CV include */
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/highgui/highgui_c.h>


#define WHITE   CV_RGB(255, 255, 255)
#define BLACK	CV_RGB(0, 0 ,0)
#define RED     CV_RGB(255, 0, 0)
#define GREEN   CV_RGB(0, 255, 0)
#define BLUE    CV_RGB(0, 0, 255)
#define YELLOW  CV_RGB(255, 255, 0)
#define PURPLE  CV_RGB(255, 0, 255)
#define GREY    CV_RGB(200, 200, 200)

#define MAX_DEPTH			4000
#define DEPTH_SCALE_FACTOR  255.f/8000.f
#define INV_DEPTH_SCALE_FACTOR	8000.f/255.f


/*
*function to calculate histogram of depth image
*/
extern void calculateHistogram(int* pHistogram, int histogramSize, const openni::VideoFrameRef& depthFrame);


/*
* open cv drawing utilities
*/
//Move from a point to other point 
inline CvPoint cvPointMove(CvPoint OrgPoint, CvPoint OrientPoint)
{
	CvPoint cvPointMoved = cvPoint(OrgPoint.x + OrientPoint.x, OrgPoint.y + OrientPoint.y);
	return cvPointMoved;
}


extern void cvDrawSetofPoints(IplImage* ImgDraw, CvPoint* points, CvScalar color, int point_Num);

extern IplImage* setImgROI_v(const IplImage* in_img, const CvSeq* contour);

#endif // _UTILITIES_H_
