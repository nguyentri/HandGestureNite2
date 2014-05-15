
#include <conio.h>
#include <stdio.h>
#include <OpenNI.h>
#include "GestureRecognition.h"
#include "PalmFinder.h"

//using namespace HandGestureCore;

#define SHOW_HAND_CONTOUR

CvPoint pt0,pt,p,armcenter;//center:´x¤ßÂI  armcenter:½ü¹ø¤¤¤ßÂI
CvRect CvRectgl;

inline float cvDistance2D(const CvPoint* p0, const CvPoint* p1);

inline float cvAngleDeg2D(const CvPoint* p0, const CvPoint* p1, const CvPoint* p2);

HandGetureTypeSt HandGestureSt = { };

void init_capture(HandGetureTypeSt *pHandGestureSt)
{
}

void init_recording(HandGetureTypeSt *pHandGestureSt)
{
	int fps = 10, width = 640, height = 480;

	pHandGestureSt->writer = cvCreateVideoWriter(VIDEO_FILE, VIDEO_FORMAT, fps,
					  cvSize(width, height), 1);

	if (!pHandGestureSt->writer) {
		fprintf(stderr, "Error initializing video writer\n");
		exit(1);
	}
}

void init_windows(void)
{
	cvNamedWindow("output", CV_WINDOW_NORMAL);
	cvNamedWindow("thresholded", CV_WINDOW_NORMAL);
	cvMoveWindow("output", 50, 50);
	cvMoveWindow("thresholded", 700, 50);
}

void init_pHandGestureSt(HandGetureTypeSt *pHandGestureSt)
{
//	pHandGestureSt->thr_image = cvCreateImage(cvSize(640, 480), 8, 1);
//	pHandGestureSt->temp_image1 = cvCreateImage(cvSize(640, 480), 8, 1);
//	pHandGestureSt->temp_image3 = cvCreateImage(cvSize(640, 480), 8, 3);
	pHandGestureSt->kernel = cvCreateStructuringElementEx(9, 9, 4, 4, CV_SHAPE_RECT,
						   NULL);

		//HandPoints = (Point*)calloc();
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

 	cvSmooth(pHandGestureSt->thr_image, pHandGestureSt->thr_image, CV_MEDIAN, 7, 7, 0.5, 0.5);

 //   cvErode(pHandGestureSt->thr_image,pHandGestureSt->thr_image,0,1);
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


 	 /*Draw rectangel */
	 if(pHandGestureSt->contour)
	 {
 		 CvRectgl = cvBoundingRect(pHandGestureSt->contour);
 		 //Draw box of hand
 		 CvPoint p1 = cvPoint(CvRectgl.x, CvRectgl.y);
 		 CvPoint p2 = cvPoint(CvRectgl.x + CvRectgl.width, CvRectgl.y + CvRectgl.height);
 		 cvRectangle(pHandGestureSt->image, p1, p2, GREEN, 1); 
	 
		armcenter.x = CvRectgl.x + CvRectgl.width/2;
        armcenter.y = CvRectgl.y + CvRectgl.height/2;

		//CvBox2D handbox = cvMinAreaRect2(pHandGestureSt->contour);
	    //           armcenter.x = cvRound(handbox.center.x);
         //       armcenter.y = cvRound(handbox.center.y);
		//CvPoint2D32f rect_pts0[4];
		//cvBoxPoints(handbox, rect_pts0);
		//int ctpoint = 4;
		//CvPoint rect_pts[4], *pt = rect_pts;
		//for (int rp=0; rp<4; rp++)
		//	rect_pts[rp]= cvPointFrom32f(rect_pts0[rp]);
		//cvPolyLine(pHandGestureSt->image, 	&pt, &ctpoint, 1, 1, RED, 1) ;
		///*Draw box */
		cvCircle(pHandGestureSt->image, armcenter, 2,
			 RED, 1, CV_AA, 0);


		//IplImage* resImg = getThreshImg(pHandGestureSt->contour, pHandGestureSt->thr_image);

		//Get list of points of threshold
		pHandGestureSt->thresholdPoints .clear();
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


	if (pHandGestureSt->hull) {

		pt0 = **CV_GET_SEQ_ELEM( CvPoint*, pHandGestureSt->hull, pHandGestureSt->hull->total - 1 );


		for(i = 0; i < pHandGestureSt->hull->total; i++ )
		{
			/*Draw convect hull */
			pt = **CV_GET_SEQ_ELEM( CvPoint*, pHandGestureSt->hull, i );
			//printf("%d,%d\n",pt.x,pt.y);
			cvLine( pHandGestureSt->image, pt0, pt, YELLOW,2,8,0);
			pt0 = pt;
		}


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
//
			free(defect_array);
		}
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

#if defined(SHOW_HAND_CONTOUR)
		cvDrawContours(pHandGestureSt->image, pHandGestureSt->contour, BLUE, GREEN, 0, 1,
			       CV_AA, cvPoint(0, 0));
#endif
		cvCircle(pHandGestureSt->image, pHandGestureSt->hand_center, 5, PURPLE, 1, CV_AA, 0);
		cvCircle(pHandGestureSt->image, pHandGestureSt->hand_center, pHandGestureSt->hand_radius,
			 RED, 1, CV_AA, 0);
 
 		for (i = 0; i < pHandGestureSt->num_fingers; i++) {
 
 			cvCircle(pHandGestureSt->image, pHandGestureSt->fingers[i], 5,
 				 GREEN, 3, CV_AA, 0);
 			cvLine(pHandGestureSt->image, pHandGestureSt->hand_center, pHandGestureSt->fingers[i],
 			       YELLOW, 1, CV_AA, 0);
 		}

		for (i = 0; i < pHandGestureSt->num_defects; i++) {
			cvCircle(pHandGestureSt->image, pHandGestureSt->defects[i], 2,
				 GREEN, 2, CV_AA, 0);
		}

		CvFont cvfont = cvFont(10, 2);
		char numfg[10];
		sprintf(numfg, "%d", pHandGestureSt->num_fingers);
		cvPutText(pHandGestureSt->image, &numfg[0], cvPoint(520, 120), &cvfont, GREEN);
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
//    int tiplocation[20];
    int count = 0;
    bool signal = false;
	pHandGestureSt->num_fingers = 0;

	uint16_t contTotal =  pHandGestureSt->Orgcontour->total;
	uint16_t contStep = pHandGestureSt->dfdisthreshold<<1;

	
	//y=3/*x+60
	float thresholdAngle = float(3)*pHandGestureSt->handDepth/200 + 60;

     for(i = 0;i < contTotal; i++)
     {
		p1 = (CvPoint*)cvGetSeqElem(pHandGestureSt->Orgcontour ,i);
		p = (CvPoint*)cvGetSeqElem(pHandGestureSt->Orgcontour ,(i + contStep)%contTotal);
		p2 = (CvPoint*)cvGetSeqElem(pHandGestureSt->Orgcontour ,(i+ (contStep<<1))%contTotal);

		 angle = cvAngleDeg2D((const CvPoint*)(p), (const CvPoint*)(p1), (const CvPoint*)(p2));

         ///////////////////////////////////if start
         if(angle < 60)
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
                CvPoint l1,l2,l3;//temp1為中心到指尖, vector1為指尖到p1, vector2為指尖到p2
                l1.x = min.x - pHandGestureSt->hand_center.x;
                l1.y = min.y - pHandGestureSt->hand_center.y;

                l2.x = minp1.x - pHandGestureSt->hand_center.x;
                l2.y = minp1.y - pHandGestureSt->hand_center.y;

                l3.x = minp2.x - pHandGestureSt->hand_center.x;
                l3.y = minp2.y - pHandGestureSt->hand_center.y;

                length = sqrtf((l1.x*l1.x)+(l1.y*l1.y));
                length1 = sqrtf((l2.x*l2.x)+(l2.y*l2.y));
                length2 = sqrtf((l3.x*l3.x)+(l3.y*l3.y));

                if(length > length1 && length > length2 && min.y < (pHandGestureSt->hand_center.y + pHandGestureSt->hand_radius) && length > 1.5*pHandGestureSt->hand_radius)
               {
                    //cvCircle(pHandGestureSt->image,min,6,CV_RGB(0,255,0),-1,8,0);
					pHandGestureSt->fingers[pHandGestureSt->num_fingers] = min;
					pHandGestureSt->num_fingers++;
                }
                else if(length < length1 && length < length2)
                {
                   cvCircle(pHandGestureSt->image,min,8, WHITE,-1,8,0);
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

#if 0
void GestureRecognition(HandGetureTypeSt *pHandGestureSt)
{
	HandGetureTypeSt* pHandGestureStp = pHandGestureSt;
	switch (pHandGestureSt->num_fingers)
	{
		case 1:
			//oneFinger(pHandGestureStp);
			break;
		case 2:
			//twoFinger(pHandGestureStp);
			break;
		case 3:
			threeFinger(pHandGestureStp);
			break;
		case 4:
			break;
		case 5:
			break;
	}
}

void threeFiger()
#endif

void findHand(HandGetureTypeSt* pHandGestureSt)
{
	int pxidx = 0;
	CvPoint handcenter;
	int handRadius;
	handcenter = pHandGestureSt->hand_center;
	handRadius =  pHandGestureSt->hand_radius;
	int ymax = handcenter.y + handRadius;

	cvCopy(pHandGestureSt->thr_image, pHandGestureSt->forcearm_img, NULL);

	for (int y = 0; y < 480; ++y)
	{
		for (int x = 0; x < 640; ++x, ++pxidx)
		{
			if((y > ymax) && (y < (ymax + 60)))
			{}
			else
			{
				pHandGestureSt->forcearm_img->imageData[pxidx] = 0;
			}
		}
//		++pxidx;
	}

	cvShowImage("handdirection", pHandGestureSt->forcearm_img);

	cvFindContours(pHandGestureSt->forcearm_img, pHandGestureSt->temp_st, &pHandGestureSt->handDirectionCont,
		       sizeof(CvContour), CV_RETR_EXTERNAL,
		       CV_CHAIN_APPROX_SIMPLE, cvPoint(0, 0));

	if(pHandGestureSt->handDirectionCont)
	{

		CvBox2D handbox = cvMinAreaRect2(pHandGestureSt->handDirectionCont);
		CvPoint2D32f rect_pts0[4];
		cvBoxPoints(handbox, rect_pts0);
		int ctpoint = 4;
		CvPoint rect_pts[4], *pt = rect_pts;
		for (int rp=0; rp<4; rp++)
			rect_pts[rp]= cvPointFrom32f(rect_pts0[rp]);
		cvPolyLine(pHandGestureSt->image, 	&pt, &ctpoint, 1, 1, RED, 1) ;
		cvLine(pHandGestureSt->image, cvPointFrom32f(handbox.center), pHandGestureSt->hand_center, PURPLE, 2, 8, 0);
	}
}

IplImage* getThreshImg(CvSeq* contour, IplImage* ThresholdImage)
{
	CvRect rect;
	rect = cvBoundingRect(contour, NULL);
	IplImage* result = cvCreateImage(cvGetSize(ThresholdImage),ThresholdImage->depth, ThresholdImage->nChannels);
	cvSetImageROI(ThresholdImage, rect);
	cvCopy(ThresholdImage,result, NULL);
	cvResetImageROI(ThresholdImage);
	cvShowImage("hand", result);
	return result;
}

std::vector<CvPoint> getListofPointofThImg(IplImage* pThImg, CvPoint handPoint)
{
	std::vector<CvPoint> poinList;
	CvScalar pointVal;
	CvPoint pointCoo;

	//uchar* pImgData = (uchar*)pThImg->imageData;

	/*int startRow = (CvRectgl.width>>1) > 20 ? armcenter.x - 20 : 0;
	int endRow = (CvRectgl.width>>1) > 20 ? armcenter.x + 20 : armcenter.x + (CvRectgl.width>>1);

	for(int idxrow = startRow; idxrow < endRow; ++idxrow)
	{
		pointCoo.x = idxrow;
		for(int idxcol = CvRectgl.y; idxcol < CvRectgl.y + CvRectgl.height; ++idxcol)
		{
			pointCoo.y = idxcol;
			pointVal = cvGet2D(pThImg, idxrow, idxcol);
			//if(pointCoo.x < armcenter.x + 10 && pointCoo.x < armcenter.x - 10  && *pImgData == 255)
			if(pointVal.val[0] != 0)
			{
				poinList.push_back(pointCoo);
			}
		}
	}*/
	poinList.clear();
	for(int idxrow = CvRectgl.x; idxrow < CvRectgl.x + CvRectgl.width; ++idxrow)
	{
		pointCoo.x = idxrow;
		for(int idxcol = CvRectgl.y; idxcol < CvRectgl.y + CvRectgl.height; ++idxcol)
		{
			pointCoo.y = idxcol;
			pointVal = cvGet2D(pThImg, idxrow, idxcol);
			//if(pointCoo.x < armcenter.x + 10 && pointCoo.x < armcenter.x - 10  && *pImgData == 255)
			if(pointVal.val[0] != 0)
			{
				poinList.push_back(pointCoo);
			}
		}
	}

	return poinList;
}