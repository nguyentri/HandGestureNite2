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

#endif // _UTILITIES_
