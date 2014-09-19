

#ifndef _HANDMOMENT_H_
#define	_HANDMOMENT_H_

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/highgui/highgui_c.h>



#define PI (double)3.14159265

float calculateTilt(const IplImage*	input_image);
float angleToCOG(CvPoint tipPt, CvPoint cogPt, float contourAxisAngle);

#endif	//end _HANDMOMENT_H_
  