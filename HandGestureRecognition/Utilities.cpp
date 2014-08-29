/*******************************************************************************
*                                                                              *
*   PrimeSense NiTE 2.0 - Common Utilities for Samples                         *
*   Copyright (C) 2012 PrimeSense Ltd.                                         *
*                                                                              *
*******************************************************************************/

#ifndef _UTILITIES_
#define _UTILITIES_

#include <stdio.h>
#include <OpenNI.h>
#include <Utilities.h>





/*
* Fuction to draw histogram of depth image
*/
void calculateHistogram(int* pHistogram, int histogramSize, const openni::VideoFrameRef& depthFrame)
{
	const openni::DepthPixel* pDepth = (const openni::DepthPixel*)depthFrame.getData();
	int* pHistogram_temp = new int[histogramSize];
	int width = depthFrame.getWidth();
	int height = depthFrame.getHeight();
	// Calculate the accumulative histogram (the yellow HandSegmentation...)
	memset(pHistogram, 0, histogramSize*sizeof(int));
	memset(pHistogram_temp, 0, histogramSize*sizeof(int));
	int restOfRow = depthFrame.getStrideInBytes() / sizeof(openni::DepthPixel) - width;

	unsigned int nNumberOfPoints = 0;
	for (int y = 0; y < height; ++y)
	{
		for (int x = 0; x < width; ++x, ++pDepth)
		{
			if (*pDepth != 0 && *pDepth <= MAX_DEPTH)
			{
				pHistogram_temp[*pDepth]++;
				nNumberOfPoints++;
			}
		}
		pDepth += restOfRow;
	}
	if (nNumberOfPoints)
	{
		for (int nIndex=1; nIndex < histogramSize; nIndex++)
		{
			pHistogram_temp[nIndex] += pHistogram_temp[nIndex-1];
			pHistogram[nIndex] = (int)(256 * (1.0f - ((float)pHistogram_temp[nIndex] / nNumberOfPoints)));
		}
	}
}


/*Function to draw a set of points */
void cvDrawSetofPoints(IplImage* ImgDraw, CvPoint* points, CvScalar color, int point_Num)
{
	CvPoint p1 = points[0];
	CvPoint p2;
	for(int idx = 1; idx < point_Num ; idx++)
	{
		p2 = points[idx];
		cvLine(ImgDraw, p1, p2, YELLOW, 2, 8, 0);
		p1 = p2;
	}
}


/*Function to element image */
IplImage* setImgROI_v(const IplImage* in_img, const CvSeq* contour)
{
	//IplImage*	out_img; 
 	CvRect	CvRectgl = cvBoundingRect((CvSeq*)contour);
	//Draw box of hand
	CvPoint p1 = cvPoint(CvRectgl.x, CvRectgl.y);
	CvPoint p2 = cvPoint(CvRectgl.x + CvRectgl.width, CvRectgl.y + CvRectgl.height);
	//create temporary image
	IplImage* img_t = cvCreateImage(cvSize(CvRectgl.width, CvRectgl.height), in_img->depth, in_img->nChannels);
	cvSetImageROI((IplImage*)in_img, CvRectgl);
	cvCopy(in_img, img_t, NULL);
	cvResetImageROI((IplImage*)in_img);
	//copy to out image
	//out_img = cvCreateImage(cvSize(CvRectgl.width, CvRectgl.height), img_t->depth, img_t->nChannels);
	//cvCopy(img_t, out_img,	NULL);	
	//release image
	//cvReleaseImage(&img_t);

	return img_t;
}





#endif // _UTILITIES_
