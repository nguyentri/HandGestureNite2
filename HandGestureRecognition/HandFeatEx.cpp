

#include <conio.h>
#include <stdio.h>
#include <math.h>

#include "HandFeatEx.h"
#include "PalmFinder.h"
#include "HandLabel.h"
#include "Utilities.h"
#include "HandGestRcg.h"

#define SHOW_HAND_CONTOUR

CvPoint pt0,pt,p;//center:

CvRect	CvRectgl;

HandGetureTypeSt HandGestureSt;

inline float cvDistance2D(const CvPoint* p0, const CvPoint* p1);

inline float cvAngleDeg2D(const CvPoint* MidP, const CvPoint* p1, const CvPoint* p2);

inline float cvDistance2D(const CvPoint* p0, const CvPoint* p1)
{
	double dist = sqrt((p1->x - p0->x)*(p1->x - p0->x) + (p1->y - p0->y)*(p1->y - p0->y));	
	return float(dist);
}

inline float cvAngleDeg2D(const CvPoint* MidP, const CvPoint* p1, const CvPoint* p2)
{
	double l1 = cvDistance2D(MidP,p1);
	double l2 = cvDistance2D(MidP,p2);
	double dot = (double)((p1->x - MidP->x) * (p2->x - MidP->x)) + (double)((p1->y - MidP->y) * (p2->y - MidP->y));
	double angle = acos(dot / (l1 * l2));
	angle = angle * 180 / (3.14159);
	return float(angle);
}

static float removeLowestPoint_F(const HandGetureTypeSt *pHandGestureSt, const CvPoint tip);

void init_capture(HandGetureTypeSt *pHandGestureSt)
{
}

void init_recording(HandGetureTypeSt *pHandGestureSt)
{
	int fps = 15, width = 190, height = 190;

	pHandGestureSt->writer = cvCreateVideoWriter(VIDEO_FILE, VIDEO_FORMAT, fps,
					  cvSize(width, height), 0);

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
		//HandGestureSt.hand_image = setImgROI_v(HandGestureSt.thr_image, HandGestureSt.contour);
		find_listPoints(&HandGestureSt);
		//cvShowImage("hand", HandGestureSt.hand_image);
		find_convex_hull(&HandGestureSt);
		FindPalm(&HandGestureSt);		
		fingertip(&HandGestureSt);
		//Display the OpenCV texture map
		//
		HandGestureSt.contourAxisAngle = calculateTilt(HandGestureSt.thr_image);
		nameFingers(&HandGestureSt);
		handRecognition();

		HandDisplay(&HandGestureSt);
		//cvReleaseImage(&HandGestureSt.hand_image);
		//cvWriteFrame(HandGestureSt.writer, HandGestureSt.image);//Write frame
}
 
void handTrainingProcessing(void)
{
		//external function called for get hand gesture
		//Call hand processing
		filter_and_threshold(&HandGestureSt);
		find_contour(&HandGestureSt);
		//HandGestureSt.hand_image = setImgROI_v(HandGestureSt.thr_image, HandGestureSt.contour);
		find_listPoints(&HandGestureSt);
		//cvShowImage("hand", HandGestureSt.hand_image);
		find_convex_hull(&HandGestureSt);
		FindPalm(&HandGestureSt);		
		fingertip(&HandGestureSt);
		//Display the OpenCV texture map
		HandGestureSt.contourAxisAngle = calculateTilt(HandGestureSt.thr_image);
		//map to training data
		//trainingData_st.finger_num_u8 = HandGestureSt.num_fingers;
		
		//for (int idx = 0; idx < trainingData_st.finger_num_u8; idx++)
		{
			//trainingData_st.angle_f[idx] = angleToCOG(HandGestureSt.fingers[idx], HandGestureSt.hand_center, HandGestureSt.contourAxisAngle);
		}
		//cvReleaseImage(&HandGestureSt.hand_image);
		//cvWriteFrame(HandGestureSt.writer, HandGestureSt.image);//Write frame	
} 
  
 
 
void init_HandGestureSt(HandGetureTypeSt *pHandGestureSt)
{
	pHandGestureSt->cvFontFingerName = cvFont(1.2, 1);
	pHandGestureSt->cvFontFingerNumber = cvFont(5, 2);

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

	pHandGestureSt->ker = cvCreateStructuringElementEx(4,4,0,0,CV_SHAPE_ELLIPSE,NULL);

	//depth_file = fopen(depth_file_name, "w" );
}


void filter_and_threshold(HandGetureTypeSt *pHandGestureSt)
{
	cvDilate(pHandGestureSt->thr_image,pHandGestureSt->thr_image, pHandGestureSt->ker, 0);  //¿±µÈ 

	cvErode(pHandGestureSt->thr_image,pHandGestureSt->thr_image, pHandGestureSt->ker, 0);

 	cvSmooth(pHandGestureSt->thr_image, pHandGestureSt->thr_image, CV_MEDIAN, 7, 7, 0, 0);
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
	cvReleaseImage(&pHandGestureSt->temp_image1);

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
}



void find_listPoints(HandGetureTypeSt *pHandGestureSt)
{
	/*Find list of point hand gesture */
	if(pHandGestureSt->contour)
	{
		//Get list of points of threshold
		pHandGestureSt->thresholdPoints.clear();
		pHandGestureSt->thresholdPoints = getListofPointofThImg(pHandGestureSt->thr_image, pHandGestureSt->HandPoint);
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
			cvPutText(pHandGestureSt->image, pHandGestureSt->number, cvPoint(480 + i*5, 120), &pHandGestureSt->cvFontFingerNumber, GREEN);
		}
//	}
	//cvShowImage("DepthImage", pHandGestureSt->image);
	//cvShowImage("ThresholdImage", pHandGestureSt->thr_image);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
void fingertip(HandGetureTypeSt *pHandGestureSt)
{
    int i, j, leng_idx = 0, t_num_fig_u8 = 0;
	float length1,length2,angle,minangle,length_tip[12] = {0}, length_figures [12] = {0};
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
				
                length_tip[leng_idx] = cvDistance2D((const CvPoint*)&min.x, (const CvPoint*)&pHandGestureSt->hand_center);

				if((angelThr < 120) 
					&&  (min.y < (pHandGestureSt->hand_center.y + 0.5*pHandGestureSt->hand_radius))					
					&& ( length_tip[leng_idx] > (1.4f*(float)pHandGestureSt->hand_radius))
					//&& (min.x != (CvRectgl.x + CvRectgl.width))
					//&& (min.y != (CvRectgl.y + CvRectgl.height))
					//&& (min.x != CvRectgl.x)
					)	
                {
                    //cvCircle(pHandGestureSt->image,min,6,CV_RGB(0,255,0),-1,8,0);
					pHandGestureSt->fingers[pHandGestureSt->num_fingers] = min;
					length_figures[pHandGestureSt->num_fingers] = length_tip[leng_idx];
					t_num_fig_u8 = ++pHandGestureSt->num_fingers;
                }
                else
                {
					if(pHandGestureSt->image != NULL)
					{		
						cvCircle(pHandGestureSt->image,cvPointMove(min, HandGestureSt.RectTopHand),8, WHITE,-1,8,0);
					}
                }

				leng_idx++;
             }

         }//else end
      }//for end

	 //remove redundent points
	 if((length_figures[0] < 2*pHandGestureSt->hand_radius) && t_num_fig_u8 == 4)
	 {	
		if(pHandGestureSt->image != NULL)
		{		
			cvCircle(pHandGestureSt->image,cvPointMove(pHandGestureSt->fingers[0], HandGestureSt.RectTopHand),8, WHITE,-1,8,0);
		}
		for(j = 0; j < t_num_fig_u8; j++)
		{
			pHandGestureSt->fingers[j] = pHandGestureSt->fingers[j+1];
		}
		pHandGestureSt->num_fingers--;
		t_num_fig_u8--;
	 }

	 //{
		// int angle = 0;
		// if (pHandGestureSt->num_fingers > 0) 
		// {
		//	for(j = 0; j < t_num_fig_u8; j++)
		//	{
		//		 angle = angleToCOG(pHandGestureSt->fingers[j], pHandGestureSt->hand_center, pHandGestureSt->contourAxisAngle);
		//		if ( (angle < MIN_LITTLE) || (angle > MAX_THUMB) )
		//		{
		//			if(pHandGestureSt->image != NULL)
		//			{
		//				cvCircle(pHandGestureSt->image,cvPointMove(pHandGestureSt->fingers[j], HandGestureSt.RectTopHand),8, WHITE,-1,8,0);
		//			}	
		//			pHandGestureSt->num_fingers--;
		//			t_num_fig_u8--;
		//		}
		//	}		
		//}
	 //}

	//if (pHandGestureSt->num_fingers > 0) //check whether any figer is open 
	//{
	//	if (pHandGestureSt->fingers[pHandGestureSt->num_fingers].y >= (pHandGestureSt->hand_center.y + 0.875*pHandGestureSt->hand_radius))
	//	{
	//		if(pHandGestureSt->image != NULL)
	//		{			
	//			cvCircle(pHandGestureSt->image,cvPointMove(pHandGestureSt->fingers[pHandGestureSt->num_fingers], HandGestureSt.RectTopHand),8, WHITE,-1,8,0);
	//		}
	//		pHandGestureSt->num_fingers--;
	//	}
	//	else if (pHandGestureSt->fingers[0].y >= (pHandGestureSt->hand_center.y + 0.875*pHandGestureSt->hand_radius))
	//	{
	//		if(pHandGestureSt->image != NULL)
	//		{				
	//			cvCircle(pHandGestureSt->image,cvPointMove(pHandGestureSt->fingers[pHandGestureSt->num_fingers], HandGestureSt.RectTopHand),8, WHITE,-1,8,0);
	//		}
	//		pHandGestureSt->num_fingers--;
	//	}
	//	else{}
	//}
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


std::vector<CvPoint> getListofPointofThImg(IplImage* pThImg, CvPoint handPoint)
{
	std::vector<CvPoint> poinList;
	poinList.clear();
	double pointVal;
//	CvPoint pointCoo;
	CvPoint	handPointShifted;

	handPointShifted.x = handPoint.x - HandGestureSt.RectTopHand.x;
	handPointShifted.y = handPoint.y - HandGestureSt.RectTopHand.y;

 	CvRectgl = cvBoundingRect((CvSeq*)HandGestureSt.contour);
	//Draw box of hand
	CvPoint p1 = cvPoint(CvRectgl.x, CvRectgl.y);
	CvPoint p2 = cvPoint(CvRectgl.x + CvRectgl.width, CvRectgl.y + CvRectgl.height);

	//cvRectangle(CvArr* img, CvPoint pt1, CvPoint pt2, CvScalar color, int thickness=1, int line_type=8, int shift=0 )
	cvRectangle(HandGestureSt.image, cvPointMove(p1, HandGestureSt.RectTopHand), cvPointMove(p2, HandGestureSt.RectTopHand), GREEN, 1); 

	int startRow = (CvRectgl.width>>1) > 20 ? handPointShifted.x - 20 : 0;
	int endRow = (CvRectgl.width>>1) > 20 ? handPointShifted.x + 20 : handPointShifted.x + (CvRectgl.width>>1);


	IplImage* temp = cvCreateImage(cvGetSize(HandGestureSt.thr_image),HandGestureSt.thr_image->depth,  HandGestureSt.thr_image->nChannels);
	cvZero(temp);

	for(int idxcol = CvRectgl.y; idxcol < CvRectgl.y + CvRectgl.height; ++idxcol)
	{
		//pointCoo.x = idxrow;
		for(int idxrow = CvRectgl.x ; idxrow < CvRectgl.x + CvRectgl.width; ++idxrow)
		{
			//pointCoo.y = idxcol;
			pointVal = cvGetReal2D(pThImg, idxrow, idxcol);
			//if(pointCoo.x < armcenter.x + 10 && pointCoo.x < armcenter.x - 10  && *pImgData == 255)
			if(pointVal == 255// && cvDistance2D(&handPointShifted, &pointCoo) < 30
				)
			{
				poinList.push_back(cvPoint(idxcol, idxrow));
				cvSetReal2D(temp, idxrow, idxcol, pointVal);
			}
			else
			{
				//poinList.push_back(cvPoint(idxrow, idxcol));
				cvSetReal2D(temp, idxrow, idxcol, 128);
			}
		}
	}

	cvShowImage("test", temp);
	cvReleaseImage(&temp);
	return poinList;
}

float removeLowestPoint_F(const HandGetureTypeSt *pHandGestureSt, const CvPoint tip)
{
	float theta = angleToCOG(pHandGestureSt->HandPoint, tip, pHandGestureSt->contourAxisAngle);
	return theta;
}