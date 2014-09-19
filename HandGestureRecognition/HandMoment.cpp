

#include "HandMoment.h"
#include "HandFeatEx.h"

float calculateTilt(const IplImage*	input_image)
  /* Return integer degree angle of contour's major axis relative to the horizontal, 
     assuming that the positive y-axis goes down the screen. 

     This code is based on maths explained in "Simple Image Analysis By Moments", by
     Johannes Kilian, March 15, 2001 (see Table 1 on p.7). 
     The paper is available at:
          http://public.cranfield.ac.uk/c5354/teaching/dip/opencv/SimpleImageAnalysisbyMoments.pdf
  */
{
	CvMoments moments;
	double m00, m10, m01, m11, m20, m02; 
	double mdiff;
	double theta;
	float tilt;
	
	cvMoments(input_image, &moments, 1);     // CvSeq is a subclass of CvArr
	//calculate moment parameters
	m00 = cvGetSpatialMoment( &moments, 0, 0) ;
	m10 = cvGetSpatialMoment( &moments, 1, 0) ;
	m01 = cvGetSpatialMoment( &moments, 0, 1);
	m11 = cvGetCentralMoment( &moments, 1, 1);
	m20 = cvGetCentralMoment( &moments, 2, 0);
	m02 = cvGetCentralMoment( &moments, 0, 2);

		if (m00 != 0) {   // calculate center
			  HandGestureSt.hand_center_mm.x = (m10/m00);
			  HandGestureSt.hand_center_mm.y  = (m01/m00);
			cvCircle(HandGestureSt.image, HandGestureSt.hand_center_mm, 2,
			RED, 1, CV_AA, 0);
		 }


    mdiff = m20 - m02;
    if (mdiff == 0) {
      if (m11 == 0)
        return 0;
      else if (m11 > 0)
        return 45;
      else   // m11 < 0
        return -45;
    }

    theta = 0.5 * atan2(2*m11, mdiff);
    tilt = (float) ((theta*180) / PI);

    if ((mdiff > 0) && (m11 == 0))
      return 0;
    else if ((mdiff < 0) && (m11 == 0))
      return -90;
    else if ((mdiff > 0) && (m11 > 0))  // 0 to 45 degrees
      return tilt;
    else if ((mdiff > 0) && (m11 < 0))  // -45 to 0
      return (180 + tilt);   // change to counter-clockwise angle measure
    else if ((mdiff < 0) && (m11 > 0))   // 45 to 90
      return tilt;
    else if ((mdiff < 0) && (m11 < 0))   // -90 to -45
      return (180 + tilt);  // change to counter-clockwise angle measure
	else
	  return 0;
}  // end of calculateTilt()



float angleToCOG(CvPoint tipPt, CvPoint cogPt, float contourAxisAngle)
  /* calculate angle of tip relative to the COG, remembering to add the
     hand contour angle so that the hand is orientated straight up */
{
	float offsetAngleTip;

    int yOffset;    // make y positive up screen
	int xOffset;

	yOffset = cogPt.y - tipPt.y;
	xOffset = tipPt.x - cogPt.x;
	/*
	if ( cogPt.x >= tipPt.x)
	{
		xOffset = tipPt.x - cogPt.x;
	}else{
		xOffset = cogPt.x - tipPt.x;
	}*/

    double theta = atan2(yOffset, xOffset);
    float angleTip = (float)theta*180/PI;
	//return angleTip;

    offsetAngleTip = angleTip + (contourAxisAngle - 90);

	//else
	//return abs(offsetAngleTip);
	return offsetAngleTip;
}  // end of angleToCOG()