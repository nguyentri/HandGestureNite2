

#include <conio.h>
#include <stdio.h>
#include "GestureRecognition.h"
#include "PalmFinder.h"
#include <math.h>


#define SHOW_HAND_CONTOUR

CvPoint pt0,pt,p,armcenter;//center:
CvRect CvRectgl;

const char* fingerName[] = {"little", "ring", "middle" ,"index", "thumb"};

const char* numberLIST[] = {"0", "1", "2", "3", "4", "5", "6", "7", "8", "9", "10"};

const CvFont cvFontFingerName = cvFont(1.2, 1);

const CvFont cvFontFingerNumber = cvFont(5, 2);

inline float cvDistance2D(const CvPoint* p0, const CvPoint* p1);

inline float cvAngleDeg2D(const CvPoint* p0, const CvPoint* p1, const CvPoint* p2);

HandGetureTypeSt HandGestureSt = { };

void init_capture(HandGetureTypeSt *pHandGestureSt)
{
}

void init_recording(HandGetureTypeSt *pHandGestureSt)
{
	int fps = 15, width = 640, height = 480;

	pHandGestureSt->writer = cvCreateVideoWriter(VIDEO_FILE, VIDEO_FORMAT, fps,
					  cvSize(width, height), 1);

	if (!pHandGestureSt->writer) {
		fprintf(stderr, "Error initializing video writer\n");
		exit(1);
	}
}

void handProcessing(void)
{
		//external function called for get hand gesture
		//Call hand processing
		filter_and_threshold(&HandGestureSt);
		find_contour(&HandGestureSt);
		find_convex_hull(&HandGestureSt);
		fingertip(&HandGestureSt);
		//find_fingers(&cvctx);
		//findHand(&HandGestureSt);
		FindPalm(&HandGestureSt);
		//Display the OpenCV texture map
		nameFingers();
		HandDisplay(&HandGestureSt);	
		cvWriteFrame(HandGestureSt.writer, HandGestureSt.image);//Write frame
}

void init_HandGestureSt(HandGetureTypeSt *pHandGestureSt)
{

	pHandGestureSt->kernel = cvCreateStructuringElementEx(9, 9, 4, 4, CV_SHAPE_RECT, NULL);
	pHandGestureSt->forcearm_img = cvCreateImage(cvSize(640, 480), 8, 1);

	pHandGestureSt->contour_st = cvCreateMemStorage(0);
	pHandGestureSt->hull_st = cvCreateMemStorage(0);
	pHandGestureSt->temp_st = cvCreateMemStorage(0);
	pHandGestureSt->handbox_str = cvCreateMemStorage(0);
	pHandGestureSt->fingers = (CvPoint *)calloc(NUM_FINGERS + 1, sizeof(CvPoint));
	pHandGestureSt->defects = (CvPoint *)calloc(NUM_DEFECTS, sizeof(CvPoint));

	pHandGestureSt->palmstorage = cvCreateMemStorage(0);
    pHandGestureSt->palm = cvCreateSeq(CV_SEQ_ELTYPE_POINT,sizeof(CvSeq),sizeof(CvPoint),pHandGestureSt->palmstorage); //Àx¦s¶ÀÂIªº§Ç¦C
	pHandGestureSt->fingerstorage = cvCreateMemStorage(0);
	pHandGestureSt->fingerseq = cvCreateSeq(CV_SEQ_ELTYPE_POINT,sizeof(CvSeq),sizeof(CvPoint),pHandGestureSt->fingerstorage);
}

void filter_and_threshold(HandGetureTypeSt *pHandGestureSt)
{
//	cvDilate(pHandGestureSt->thr_image,pHandGestureSt->thr_image,0, 0);  //¿±µÈ 

 	cvSmooth(pHandGestureSt->thr_image, pHandGestureSt->thr_image, CV_MEDIAN, 7, 7, 0, 0);

 //  cvErode(pHandGestureSt->thr_image,pHandGestureSt->thr_image,0,1);
}


void find_contour(HandGetureTypeSt *pHandGestureSt)
{
	double area, max_area = 0.0;
	CvSeq *contours, *tmp, *contour = NULL;

	/* cvFindContours modifies input image, so make a copy */
	pHandGestureSt->temp_image1 = cvCreateImage(cvGetSize(pHandGestureSt->thr_image), pHandGestureSt->thr_image->depth,  pHandGestureSt->thr_image->nChannels);
	cvCopy(pHandGestureSt->thr_image, pHandGestureSt->temp_image1, NULL);
	cvFindContours(pHandGestureSt->temp_image1, pHandGestureSt->temp_st, &contours,
		       sizeof(CvContour), CV_RETR_EXTERNAL,
		       CV_CHAIN_APPROX_SIMPLE, cvPoint(0, 0));

	/* Select contour having greatest area */
	for (tmp = contours; tmp; tmp = tmp->h_next) {
		area = fabs(cvContourArea(tmp, CV_WHOLE_SEQ, 0));
		if (area > max_area) {
			max_area = area;
			contour = tmp;
		}
	}

	/* Approximate contour with poly-line */
	if (contour) {
		pHandGestureSt->Orgcontour = contour;
		contour = cvApproxPoly(contour, sizeof(CvContour),
				       pHandGestureSt->contour_st, CV_POLY_APPROX_DP, 2,
				       1);
		pHandGestureSt->contour = contour;
	}


	/*Draw hand box */
	if(pHandGestureSt->contour)
	{
		
 		CvRectgl = cvBoundingRect(pHandGestureSt->contour);
		//Draw box of hand
		CvPoint p1 = cvPoint(CvRectgl.x, CvRectgl.y);
		CvPoint p2 = cvPoint(CvRectgl.x + CvRectgl.width, CvRectgl.y + CvRectgl.height);
		cvRectangle(pHandGestureSt->image, cvPointMove(p1, HandGestureSt.RectTopHand), cvPointMove(p2, HandGestureSt.RectTopHand), GREEN, 1); 
	 
		/*armcenter.x = CvRectgl.x + CvRectgl.width/2;
		armcenter.y = CvRectgl.y + CvRectgl.height/2;

		CvBox2D handbox = cvMinAreaRect2(pHandGestureSt->contour);
				armcenter.x = cvRound(handbox.center.x);
			armcenter.y = cvRound(handbox.center.y);
		CvPoint2D32f rect_pts0[4];
		cvBoxPoints(handbox, rect_pts0);
		int ctpoint = 4;
		CvPoint rect_pts[4], *pt = rect_pts;
		for (int rp=0; rp<4; rp++)
		rect_pts[rp]= cvPointFrom32f(rect_pts0[rp]);
		cvPolyLine(pHandGestureSt->image, 	&pt, &ctpoint, 1, 1, RED, 1) ;
		///Draw hand rectangle
		cvCircle(pHandGestureSt->image, armcenter, 2,
			RED, 1, CV_AA, 0); */
	
		//Get moments
		 CvMoments moments;
		 pHandGestureSt->temp_image3 = cvCreateImage(cvGetSize(pHandGestureSt->thr_image), pHandGestureSt->thr_image->depth,  pHandGestureSt->thr_image->nChannels);
		 cvDilate(pHandGestureSt->thr_image,pHandGestureSt->temp_image3, 0, 6);
		 cvErode(pHandGestureSt->temp_image3,pHandGestureSt->temp_image3, 0 , 18);
		// cvShowImage("error image",pHandGestureSt->temp_image3);
		 cvMoments(pHandGestureSt->thr_image, &moments, 1);     // CvSeq is a subclass of CvArr
		// center of gravity
		double m00 = cvGetSpatialMoment( &moments, 0, 0) ;
	    double m10 = cvGetSpatialMoment( &moments, 1, 0) ;
		double m01 = cvGetSpatialMoment( &moments, 0, 1);
		if (m00 != 0) {   // calculate center
			 armcenter.x  =(m10/m00);
			 armcenter.y = (m01/m00);
			cvCircle(pHandGestureSt->image, armcenter, 2,
			RED, 1, CV_AA, 0);
		 }
		 double m11 = cvGetCentralMoment( &moments, 1, 1);
		 double m20 = cvGetCentralMoment( &moments, 2, 0);
		 double m02 = cvGetCentralMoment( &moments, 0, 2);
		 pHandGestureSt->contourAxisAngle = calculateTilt(m11, m20, m02);
          /* this angle assumes that the positive y-axis
             is down the screen */

		//Get list of points of threshold
		pHandGestureSt->thresholdPoints.clear();
		pHandGestureSt->thresholdPoints = getListofPointofThImg(pHandGestureSt->thr_image, armcenter);
		//cvSaveImage("ThresholdImage.JPG", resImg);
	}
}

void find_convex_hull(HandGetureTypeSt *pHandGestureSt)
{
	CvSeq *defects;
	CvConvexityDefect *defect_array;
	CvConvexityDefect *efdf;
	int i;
	int x = 0, y = 0;
	int dx = 0, dy = 0;
	int sx = 0, sy = 0;
	int ex = 0, ey = 0;
	int dist = 0;

	pHandGestureSt->hull = NULL;

	if (!pHandGestureSt->contour)
		return;

	pHandGestureSt->hull = cvConvexHull2(pHandGestureSt->contour, pHandGestureSt->hull_st, CV_CLOCKWISE, 0);

		/* Get convexity defects of contour w.r.t. the convex hull */
		defects = cvConvexityDefects(pHandGestureSt->contour, pHandGestureSt->hull,
					     pHandGestureSt->defects_st);

		if (defects && defects->total) {
			defect_array = (CvConvexityDefect *)calloc(defects->total,
					      sizeof(CvConvexityDefect));
			efdf = (CvConvexityDefect *)calloc(defects->total,
				sizeof(CvConvexityDefect));
			cvCvtSeqToArray(defects, defect_array, CV_WHOLE_SEQ);

			/* Average depth points to get hand center */
			pHandGestureSt->num_defects  = 0;
			memset(pHandGestureSt->defects, 0, NUM_DEFECTS*sizeof(CvPoint)); //clear defect points
			for (i = 0; i < defects->total; i++) {
				dx = defect_array[i].depth_point->x;
				dy = defect_array[i].depth_point->y;
				x += dx;
				y += dy;
				float angle = getAngle(defect_array[i].start,  defect_array[i].depth_point, defect_array[i].end);

				if (defect_array[i].depth > pHandGestureSt->dfdisthreshold  && angle < 135) // remove defects <= 10mm
				{
					pHandGestureSt->defects[pHandGestureSt->num_defects] = cvPoint(dx, dy);
					pHandGestureSt->num_defects++;
				}
			}

			free(defect_array);
		}
}

void find_fingers(HandGetureTypeSt *pHandGestureSt)
{
	int n;
	int i;
	CvPoint *points;
	CvPoint max_point;
	int dist1 = 0, dist2 = 0;

	pHandGestureSt->num_fingers = 0;

	if (!pHandGestureSt->contour || !pHandGestureSt->hull)
		return;

	n = pHandGestureSt->contour->total;
	points = (CvPoint *)calloc(n, sizeof(CvPoint));

	cvCvtSeqToArray(pHandGestureSt->contour, points, CV_WHOLE_SEQ);

	/*
	 * Fingers are detected as points where the distance to the center
	 * is a local maximum
	 */
	for (i = 0; i < n; i++) {
		int dist;
		int cx = pHandGestureSt->hand_center.x;
		int cy = pHandGestureSt->hand_center.y;

		dist = (cx - points[i].x) * (cx - points[i].x) +
		    (cy - points[i].y) * (cy - points[i].y);

		if (dist < dist1 && dist1 > dist2 && max_point.x != 0
		    && max_point.y < cvGetSize(pHandGestureSt->image).height - 10) {

			pHandGestureSt->fingers[pHandGestureSt->num_fingers++] = max_point;
			if (pHandGestureSt->num_fingers >= NUM_FINGERS + 1)
				break;
		}

		dist2 = dist1;
		dist1 = dist;
		max_point = points[i];
	}

	free(points);
}

void HandDisplay(HandGetureTypeSt *pHandGestureSt)
{
	int i;

//	if (pHandGestureSt->num_fingers == NUM_FINGERS) {

	if (pHandGestureSt->hull) {

		pt0 = **CV_GET_SEQ_ELEM( CvPoint*, pHandGestureSt->hull, pHandGestureSt->hull->total - 1 );


		for(i = 0; i < pHandGestureSt->hull->total; i++ )
		{
			/*Draw convect hull */
			pt = **CV_GET_SEQ_ELEM( CvPoint*, pHandGestureSt->hull, i );
			//printf("%d,%d\n",pt.x,pt.y);
			cvLine( pHandGestureSt->image, cvPointMove(pt0,  pHandGestureSt->RectTopHand), cvPointMove(pt,  pHandGestureSt->RectTopHand), YELLOW,2,8,0);
			pt0 = pt;
		}
	}

	if (pHandGestureSt->contour) {

		pt0 = * (CvPoint*)cvGetSeqElem(pHandGestureSt->contour, 0);


		for(i = 1; i < pHandGestureSt->contour->total; i++ )
		{
			/*Draw convect hull */
			pt =  * (CvPoint*)cvGetSeqElem(pHandGestureSt->contour, i);
			//printf("%d,%d\n",pt.x,pt.y);
			cvLine( pHandGestureSt->image, cvPointMove(pt0,  pHandGestureSt->RectTopHand), cvPointMove(pt,  pHandGestureSt->RectTopHand), BLUE,2,8,0);
			pt0 = pt;
		}
	}

	//	cvDrawContours(pHandGestureSt->image, pHandGestureSt->contour, BLUE, GREEN, 0, 1,
	//		       CV_AA, cvPoint(0, 0));
		cvCircle(pHandGestureSt->image, cvPointMove(pHandGestureSt->hand_center, pHandGestureSt->RectTopHand), 5, PURPLE, 1, CV_AA, 0);
		cvCircle(pHandGestureSt->image, cvPointMove(pHandGestureSt->hand_center, pHandGestureSt->RectTopHand), pHandGestureSt->hand_radius,
			 RED, 1, CV_AA, 0);
 
 		for (i = 0; i < pHandGestureSt->num_fingers; i++) {
 
 			cvCircle(pHandGestureSt->image, cvPointMove(pHandGestureSt->fingers[i], pHandGestureSt->RectTopHand), 5,
 				 GREEN, 3, CV_AA, 0);
 			cvLine(pHandGestureSt->image, cvPointMove(pHandGestureSt->hand_center, pHandGestureSt->RectTopHand), cvPointMove(pHandGestureSt->fingers[i], pHandGestureSt->RectTopHand),
 			       YELLOW, 1, CV_AA, 0);
 		}

		for (i = 0; i < pHandGestureSt->num_defects; i++) {
			cvCircle(pHandGestureSt->image, cvPointMove(pHandGestureSt->defects[i], pHandGestureSt->RectTopHand), 2,
				 GREEN, 2, CV_AA, 0);
		}

		for (i = 0; i < strlen(pHandGestureSt->number); i++)
		{
			cvPutText(pHandGestureSt->image, pHandGestureSt->number, cvPoint(480 + i*5, 120), &cvFontFingerNumber, GREEN);
		}
//	}
	cvShowImage("DepthImage", pHandGestureSt->image);
	cvShowImage("ThresholdImage", pHandGestureSt->thr_image);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
void fingertip(HandGetureTypeSt *pHandGestureSt)
{
    int i;
    float length1,length2,angle,minangle,length;
    CvPoint min,minp1,minp2;
    CvPoint *p1,*p2,*p;
    int count = 0;
    bool signal = false;
	pHandGestureSt->num_fingers = 0;

	uint16_t contTotal =  pHandGestureSt->Orgcontour->total;
	uint16_t contStep = pHandGestureSt->dfdisthreshold<<1;

	//y=3/200*x+60
	float thresholdAngle = float(3)*pHandGestureSt->handDepth/200 + 60;

     for(i = 0;i < contTotal; i++)
     {
		p1 = (CvPoint*)cvGetSeqElem(pHandGestureSt->Orgcontour ,i);
		p = (CvPoint*)cvGetSeqElem(pHandGestureSt->Orgcontour ,(i + contStep)%contTotal);
		p2 = (CvPoint*)cvGetSeqElem(pHandGestureSt->Orgcontour ,(i+ (contStep<<1))%contTotal);

		 angle = cvAngleDeg2D((const CvPoint*)(p), (const CvPoint*)(p1), (const CvPoint*)(p2));

         ///////////////////////////////////if start
         if(angle < thresholdAngle)
         {
            if(!signal)//  if it is the first point
            {
                signal = true;
                min.x = p->x;
                min.y = p->y;
                minp1.x = p1->x;
                minp1.y = p1->y;
                minp2.x = p2->x;
                minp2.y = p2->y;
                minangle = angle;
            }
            else//  not the first point
            {
                if(angle <= minangle)//angle
                {

                    min.x = p->x;
                    min.y = p->y;
                    minp1.x = p1->x;
                    minp1.y = p1->y;
                    minp2.x = p2->x;
                    minp2.y = p2->y;
                    minangle = angle;
                }
            }

         }//////////////////////////////////////////////////////////if end
         else //else start
         {
             if(signal)
             {
                signal = false;
                CvPoint l1,l2,l3;//temp1
				float angelThr = cvAngleDeg2D(&min, &minp2, &pHandGestureSt->hand_center);
                l1.x = min.x - pHandGestureSt->hand_center.x;
                l1.y = min.y - pHandGestureSt->hand_center.y;

                l2.x = minp1.x - pHandGestureSt->hand_center.x;
                l2.y = minp1.y - pHandGestureSt->hand_center.y;

                l3.x = minp2.x - pHandGestureSt->hand_center.x;
                l3.y = minp2.y - pHandGestureSt->hand_center.y;

                length = sqrtf((l1.x*l1.x)+(l1.y*l1.y));
                length1 = sqrtf((l2.x*l2.x)+(l2.y*l2.y));
                length2 = sqrtf((l3.x*l3.x)+(l3.y*l3.y));


                if(angelThr < 90 && min.y < (pHandGestureSt->hand_center.y + (pHandGestureSt->hand_radius>>1)) && length > 1.5*pHandGestureSt->hand_radius)
               {
                    //cvCircle(pHandGestureSt->image,min,6,CV_RGB(0,255,0),-1,8,0);
					pHandGestureSt->fingers[pHandGestureSt->num_fingers] = min;
					pHandGestureSt->num_fingers++;
                }
                else
                {
                   cvCircle(pHandGestureSt->image,cvPointMove(min, HandGestureSt.RectTopHand),8, WHITE,-1,8,0);
                }
             }
         }//else end
      }//for end
}

inline float cvDistance2D(const CvPoint* p0, const CvPoint* p1)
{
	double dist = sqrt((p1->x - p0->x)*(p1->x - p0->x) + (p1->y - p0->y)*(p1->y - p0->y));	
	return float(dist);
}

inline float cvAngleDeg2D(const CvPoint* p0, const CvPoint* p1, const CvPoint* p2)
{
	double l1 = cvDistance2D(p0,p1);
	double l2 = cvDistance2D(p0,p2);
	double dot = (double)((p1->x - p0->x) * (p2->x - p0->x)) + (double)((p1->y - p0->y) * (p2->y - p0->y));
	double angle = acos(dot / (l1 * l2));
	angle = angle * 180 / (3.14159);
	return float(angle);
}

void FindPalm(HandGetureTypeSt *pHandGestureSt)
{
	Palm* palmPointer;

	PalmFinder palmFinderObj(pHandGestureSt); 

	palmPointer = palmFinderObj.FindCenter();

	pHandGestureSt->hand_center.x = palmPointer->getLocation().x;
	pHandGestureSt->hand_center.y = palmPointer->getLocation().y;
	pHandGestureSt->hand_radius = palmPointer->getDistanceToContour();
}

IplImage* getThreshImg(CvSeq* contour, IplImage* ThresholdImage)
{
	CvRect rect;
	rect = cvBoundingRect(contour, NULL);
	IplImage* result = cvCreateImage(cvGetSize(ThresholdImage),ThresholdImage->depth, ThresholdImage->nChannels);
	cvSetImageROI(ThresholdImage, rect);
	cvCopy(ThresholdImage,result, NULL);
	cvResetImageROI(ThresholdImage);
	return result;
}

std::vector<CvPoint> getListofPointofThImg(IplImage* pThImg, CvPoint handPoint)
{
	std::vector<CvPoint> poinList;
	poinList.clear();
	double pointVal;
	CvPoint pointCoo;

	//uchar* pImgData = (uchar*)pThImg->imageData;

	int startRow = (CvRectgl.width>>1) > 20 ? armcenter.x - 20 : 0;
	int endRow = (CvRectgl.width>>1) > 20 ? armcenter.x + 20 : armcenter.x + (CvRectgl.width>>1);

	for(int idxrow = startRow; idxrow < endRow; ++idxrow)
	{
		pointCoo.x = idxrow;
		for(int idxcol = CvRectgl.y; idxcol < CvRectgl.y + CvRectgl.height; ++idxcol)
		{
			pointCoo.y = idxcol;
			pointVal = cvGetReal2D(pThImg, idxrow, idxcol);
			//if(pointCoo.x < armcenter.x + 10 && pointCoo.x < armcenter.x - 10  && *pImgData == 255)
			if(pointVal != 0 && cvDistance2D(&armcenter, &pointCoo) < 30)
			{
				poinList.push_back(pointCoo);
			}
		}
	}
	return poinList;
}

float calculateTilt(double m11, double m20, double m02)
  /* Return integer degree angle of contour's major axis relative to the horizontal, 
     assuming that the positive y-axis goes down the screen. 

     This code is based on maths explained in "Simple Image Analysis By Moments", by
     Johannes Kilian, March 15, 2001 (see Table 1 on p.7). 
     The paper is available at:
          http://public.cranfield.ac.uk/c5354/teaching/dip/opencv/SimpleImageAnalysisbyMoments.pdf
  */
  {
    double diff = m20 - m02;
    if (diff == 0) {
      if (m11 == 0)
        return 0;
      else if (m11 > 0)
        return 45;
      else   // m11 < 0
        return -45;
    }

    double theta = 0.5 * atan2(2*m11, diff);
    int tilt = (float) (theta*180 / PI);

    if ((diff > 0) && (m11 == 0))
      return 0;
    else if ((diff < 0) && (m11 == 0))
      return -90;
    else if ((diff > 0) && (m11 > 0))  // 0 to 45 degrees
      return tilt;
    else if ((diff > 0) && (m11 < 0))  // -45 to 0
      return (180 + tilt);   // change to counter-clockwise angle measure
    else if ((diff < 0) && (m11 > 0))   // 45 to 90
      return tilt;
    else if ((diff < 0) && (m11 < 0))   // -90 to -45
      return (180 + tilt);  // change to counter-clockwise angle measure
    return 0;
  }  // end of calculateTilt()

void nameFingers(void)
  /* Use the finger tip coordinates, and the comtour's COG and axis angle to horizontal
     to label the fingers.

     Try to label the thumb and index based on their likely angle ranges
     relative to the COG. This assumes that the thumb and index finger are on the
     left side of the hand.

     Then label the other fingers based on the order of the names in the FingerName class
  */
  { // reset all named fingers to unknown

	std::vector<FingerNameE>namedFingers = HandGestureSt.namedFingers;

    namedFingers.clear();
    for (int i=0; i < HandGestureSt.num_fingers; i++)
		namedFingers.push_back(UNKNOWN);

    labelFingers();

    // printFingers("named fingers", namedFingers);
    //labelUnknowns(namedFingers);
    // printFingers("revised named fingers", namedFingers);
  }  // end of nameFingers()

void labelFingers(void)
  // attempt to label the thumb and index fingers of the hand
{ 
    bool foundThumb = false;
    bool foundIndex = false;
	bool foundMiddle = false;
	bool foundRing = false;
	bool foundLittle = false;
	float angleThr = 0;

      /* the thumb and index fingers will most likely be stored at the end
         of the list, since the contour hull was built in a counter-clockwise 
         order by the call to cvConvexHull2() in findFingerTips(), and I am assuming
         the thumb is on the left of the hand.
         So iterate backwards through the list.
      */
	float contourAxisAngle = 180  - HandGestureSt.contourAxisAngle;
    int i = HandGestureSt.num_fingers - 1;
    while ((i >= 0)) {
		int angle = angleToCOG(HandGestureSt.fingers[i], HandGestureSt.hand_center, contourAxisAngle);

      // check for thumb
      if ((angle <=  MAX_THUMB) && (angle > MIN_THUMB) && !foundThumb) {
		cvPutText(HandGestureSt.image, "Thum", cvPointMove(HandGestureSt.fingers[i], HandGestureSt.RectTopHand), &cvFontFingerName, PURPLE);
		  HandGestureSt.namedFingers.assign(i, THUMB);
        foundThumb = true;
      }

      // check for index
      if ((angle <= MAX_INDEX) && (angle > MIN_INDEX) && !foundIndex) {
        HandGestureSt.namedFingers.assign(i, INDEX);
		cvPutText(HandGestureSt.image, "Index", cvPointMove(HandGestureSt.fingers[i], HandGestureSt.RectTopHand), &cvFontFingerName, PURPLE);
        foundIndex = true;
      }

      // check for middle
      if ((angle <= MAX_MIDDLE) && (angle > MIN_MIDDLE) && !foundMiddle) {
        HandGestureSt.namedFingers.assign(i, MIDDLE);
		cvPutText(HandGestureSt.image, "Middle",cvPointMove(HandGestureSt.fingers[i], HandGestureSt.RectTopHand), &cvFontFingerName, PURPLE);
        foundMiddle = true;
      }

      // check for ring
      if ((angle <= MAX_RING) && (angle > MIN_RING) && !foundRing) {
        HandGestureSt.namedFingers.assign(i, RING);
		cvPutText(HandGestureSt.image, "Ring", cvPointMove(HandGestureSt.fingers[i], HandGestureSt.RectTopHand), &cvFontFingerName, PURPLE);
        foundRing = true;
      }

      // check for little
      if ((angle <= MAX_LITTLE) && (angle > MIN_LITTLE) && !foundLittle) {
        HandGestureSt.namedFingers.assign(i, MIDDLE);
		cvPutText(HandGestureSt.image, "Little", cvPointMove(HandGestureSt.fingers[i], HandGestureSt.RectTopHand), &cvFontFingerName, PURPLE);
        foundLittle = true;
      }

      i--;
    }

	switch (HandGestureSt.num_fingers)
	{
		case 0:
			strcpy(HandGestureSt.number, numberLIST[0]);
		break;
		case 1:
			if(foundIndex == true)
				strcpy(HandGestureSt.number, numberLIST[1]);
			else if(foundThumb == true)
				strcpy(HandGestureSt.number, numberLIST[10]);
			else 
			{
				strcpy(HandGestureSt.number, "N.A");
			}
		break;
		case 2:
			strcpy(HandGestureSt.number, numberLIST[2]);
		break;
		case 3:
			if(foundThumb && foundIndex && foundMiddle)
			{
				strcpy(HandGestureSt.number, numberLIST[3]);
			}
			else if(foundIndex && foundMiddle && foundRing)
			{
				strcpy(HandGestureSt.number, numberLIST[6]);
			}
			else if(foundLittle && foundMiddle && foundIndex)
			{
				strcpy(HandGestureSt.number, numberLIST[7]);
			}
			else if(foundLittle && foundRing && foundIndex)
			{
				strcpy(HandGestureSt.number, numberLIST[8]);
			}
			else if(foundLittle && foundRing && foundMiddle)
			{
				strcpy(HandGestureSt.number, numberLIST[9]);
			}
			else 
			{
				strcpy(HandGestureSt.number, "N.A");
			}
		break;
		case 4:
			strcpy(HandGestureSt.number, numberLIST[4]);
		break;
		case 5:
			strcpy(HandGestureSt.number, numberLIST[5]);
		break;
		default:
			strcpy(HandGestureSt.number, numberLIST[5]);
		break;
	}
}  // end of labelFingers()

float angleToCOG(CvPoint tipPt, CvPoint cogPt, int contourAxisAngle)
  /* calculate angle of tip relative to the COG, remembering to add the
     hand contour angle so that the hand is orientated straight up */
  {
    int yOffset = cogPt.y - tipPt.y;    // make y positive up screen
    int xOffset = tipPt.x - cogPt.x;
    // CvPoint offsetPt = new CvPoint(xOffset, yOffset);

    double theta = atan2(yOffset, xOffset);
    float angleTip = (float)theta*180/PI;
	//return angleTip;
    int offsetAngleTip = angleTip + (90 - contourAxisAngle);
             // this addition ensures that the hand is orientated straight up
    return offsetAngleTip;
  }  // end of angleToCOG()

inline CvPoint cvPointMove(CvPoint OrgPoint, CvPoint OrientPoint)
{
	CvPoint cvPointMoved = cvPoint(OrgPoint.x + OrientPoint.x, OrgPoint.y + OrientPoint.y);
	return cvPointMoved;
}

IplImage* getHand_pImg(IplImage* thr_image)
{
	IplImage* img_t = cvCreateImage(cvSize(CvRectgl.width, CvRectgl.height), thr_image->depth, thr_image->nChannels);
	cvSetImageROI(thr_image, CvRectgl);
	cvCopy(thr_image, img_t, NULL);
	cvResetImageROI(thr_image);

	//return hand image
	return img_t;
}