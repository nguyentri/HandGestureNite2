#include <iostream>
#include <vector>
#include <conio.h>
#include <stdio.h>
#include <OpenNI.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/highgui/highgui_c.h>


#define C_DEPTH_STREAM 0
#define C_COLOR_STREAM 1

#define C_NUM_STREAMS 2

#define C_MODE_COLOR 0x01
#define C_MODE_DEPTH 0x02
#define C_MODE_ALIGNED 0x04

#define C_STREAM_TIMEOUT 2000

#define MAX_DEPTH	4000

extern float m_pDepthHist[MAX_DEPTH];

#ifndef _OPENCV_HAND_UTILITIES_
#define _OPENCV_HAND_UTILITIES_

#define VIDEO_FILE	"video.avi"
#define VIDEO_FORMAT	CV_FOURCC('M', 'J', 'P', 'G')
#define NUM_FINGERS 6 
#define NUM_DEFECTS	5

#define PI 3.14159


#define WHITE   CV_RGB(255, 255, 255)
#define BLACK	CV_RGB(0, 0 ,0)
#define RED     CV_RGB(255, 0, 0)
#define GREEN   CV_RGB(0, 255, 0)
#define BLUE    CV_RGB(0, 0, 255)
#define YELLOW  CV_RGB(255, 255, 0)
#define PURPLE  CV_RGB(255, 0, 255)
#define GREY    CV_RGB(200, 200, 200)


// angle ranges of thumb and index finger of the left hand relative to its COG
#define MIN_THUMB  135
#define MAX_THUMB  200

#define MIN_INDEX  95
#define MAX_INDEX  135

#define MIN_MIDDLE 80
#define MAX_MIDDLE 95

#define MIN_RING   65
#define MAX_RING   80

#define MAX_LITTLE  65
#define MIN_LITTLE   0

enum FingerNameE {
	LITTLE, RING, MIDDLE, INDEX, THUMB, UNKNOWN};


inline float distanceP2P(const CvPoint* a, const CvPoint* b){
	float d= sqrt(fabs( pow(a->
		x-b->x,2) + pow(a->y-b->y,2) )) ;  
	return d;
}

inline float getAngle(const CvPoint* s,const CvPoint* f, CvPoint* e){
	float l1 = distanceP2P(f,s);
	float l2 = distanceP2P(f,e);
	float dot=(s->x-f->x)*(e->x-f->x) + (s->y-f->y)*(e->y-f->y);
	float angle = acos(dot/(l1*l2));
	angle = (float)angle*180/PI;
	return angle;
}

typedef struct HandGesture {
	CvCapture	*capture;	/* Capture handle */
	CvVideoWriter	*writer;	/* File recording handle */

	IplImage	*frame;
	IplImage	*image;		/* Input image */
	IplImage	*thr_image;	/* After filtering and thresholding */
	IplImage	*temp_image1;	/* Temporary image (1 channel) */
	IplImage	*temp_image3;	/* Temporary image (3 channels) */
	IplImage	*forcearm_img;


	CvSeq		*contour;	/* Hand contour after approxi */
	CvSeq		*Orgcontour; /* hand orignal countour */
	CvSeq		*hull;		/* Hand convex hull */

	CvSeq *handDirectionCont;

	CvPoint		hand_center;
	CvPoint		*fingers;	/* Detected fingers positions */
	CvPoint		*defects;	/* Convexity defects depth points */

	CvMemStorage	*hull_st;
	CvMemStorage	*contour_st;
	CvMemStorage	*temp_st;
	CvMemStorage	*defects_st;
	CvMemStorage	*handbox_str;
				
	IplConvKernel	*kernel;	/* Kernel for morph operations */

    CvSeq* palm; //palm seq
	CvMemStorage* palmstorage;

	CvSeq* fingerseq;
	CvMemStorage* fingerstorage;

	int		num_fingers;
	int		hand_radius;
	int		num_defects;
	int		dfdisthreshold;
	int		handDepth;
	float contourAxisAngle;

	std::vector<FingerNameE>namedFingers;

	std::vector<CvPoint> thresholdPoints;

	char number[20];

	CvPoint HandPoint;
} HandGetureTypeSt;

extern HandGetureTypeSt  HandGestureSt;

/*Function to initialize recording video. */
extern void init_recording(HandGetureTypeSt *pHandGestureSt);

/*Function to initialize image windows. */
extern void init_windows(void);

/*Function to initialize HandGetureTypeSt. */
extern void init_HandGestureSt(HandGetureTypeSt *pHandGestureSt);

extern void filter_and_threshold(HandGetureTypeSt *pHandGestureSt);

extern void find_contour(HandGetureTypeSt *pHandGestureSt);

extern void find_convex_hull(HandGetureTypeSt *pHandGestureSt);

extern void find_fingers(HandGetureTypeSt *pHandGestureSt);

extern void HandDisplay(HandGetureTypeSt *pHandGestureSt);

//other unilities
void fingertip(HandGetureTypeSt* pHandGestureSt);

void FindPalm(HandGetureTypeSt *pHandGestureSt);

void findHand(HandGetureTypeSt* pHandGestureSt);

IplImage* getThreshImg(CvSeq* contour, IplImage* ThresholdImage);

std::vector<CvPoint> getListofPointofThImg(IplImage* pThImg, CvPoint handPoint);

float calculateTilt(double m11, double m20, double m02);

void labelFingers();

float angleToCOG(CvPoint tipPt, CvPoint cogPt, int contourAxisAngle);

void handProcessing(void);

void nameFingers(void);

void labelFingers(void);

#endif