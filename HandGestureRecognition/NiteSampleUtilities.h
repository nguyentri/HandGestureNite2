/*******************************************************************************
*                                                                              *
*   PrimeSense NiTE 2.0 - Common Utilities for Samples                         *
*   Copyright (C) 2012 PrimeSense Ltd.                                         *
*                                                                              *
*******************************************************************************/

#ifndef _NITE_SAMPLE_UTILITIES_H_
#define _NITE_SAMPLE_UTILITIES_H_

#include <stdio.h>
#include <OpenNI.h>

#define RED     CV_RGB(255, 0, 0)
#define GREEN   CV_RGB(0, 255, 0)
#define BLUE    CV_RGB(0, 0, 255)
#define YELLOW  CV_RGB(255, 255, 0)
#define PURPLE  CV_RGB(255, 0, 255)
#define GREY    CV_RGB(200, 200, 200)

const float DEPTH_SCALE_FACTOR = 255./4096.;


void calculateHistogram(int* pHistogram, int histogramSize, const openni::VideoFrameRef& depthFrame)
{
	const openni::DepthPixel* pDepth = (const openni::DepthPixel*)depthFrame.getData();
	int* pHistogram_temp = new int[histogramSize];
	int width = depthFrame.getWidth();
	int height = depthFrame.getHeight();
	// Calculate the accumulative histogram (the yellow display...)
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

#endif // _NITE_SAMPLE_UTILITIES_H_
