/*******************************************************************************
*                                                                              *
*   PrimeSense NiTE 2.0 - Hand Viewer Sample                                   *
*   Copyright (C) 2012 PrimeSense Ltd.                                         *
*                                                                              *
*******************************************************************************/

#ifndef _NITE_HAND_GESTURE_H_
#define _NITE_HAND_GESTURE_H_

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/highgui/highgui_c.h>

#include "OpenNI.h"
#include "NiTE.h"

class HandGesture
{
public:
	HandGesture(const char* strSampleName);
	virtual ~HandGesture();

protected:
	virtual void Display();
	virtual void DisplayPostDraw(){};	// Overload to draw over the screen image

	virtual void OnKey(unsigned char key, int x, int y);

	virtual openni::Status InitOpenCV(int argc, char **argv);

	void Finalize();



private:
 	IplImage	*pHandImg;

	CvSeq		*appContour; /* Hand contour after approxi */
	CvSeq		*orgContour; /* Hand orignal countour */
	CvSeq		*hull;		 /* Hand convex hull */

	CvPoint		handCenter; /*Hand center */
	CvPoint		*fingers;	/* Detected fingers positions */
	CvPoint		*defects;	/* Convexity defects depth points */

	CvMemStorage	*hull_st;
	CvMemStorage	*contour_st;
	CvMemStorage	*temp_st;
	CvMemStorage	*defects_st;
				
	IplConvKernel	*filerKernel;	/* Kernel for morph operations */

    CvSeq* palm; //palm seq
	CvMemStorage* palmstorage;

	CvSeq* fingerseq;
	CvMemStorage* fingerstorage;

	int		num_fingers;
	int		hand_radius;
	int		num_defects;
	int		dfdisthreshold;
	std::vector<CvPoint> thresholdPoints;
};


#endif // _NITE_HAND_GESTURE_H_
