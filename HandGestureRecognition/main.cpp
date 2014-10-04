/*******************************************************************************
*                                                                              *
*   PrimeSense NiTE 2.0 - Hand HandViewer Sample                                   *
*   Copyright (C) 2012 PrimeSense Ltd.                                         *
*                                                                              *
*******************************************************************************/

#include "HandViewer.h"
#include "HandSegm.h"
#include "HandFeatEx.h"
#include "HandTranning.h"
#include "HandGestRcg.h"

#define GETSAMPLE

int main(int argc, char** argv)
{
	int key;

	/*Init camera */
	openni::Status rc = openni::STATUS_OK;
	/*Declare objects */

	HandViewer HandViewerObj("Hand Geture", "Main Window", "Threshold Image");
	HandSegm HandSegmObj;

	/*Init kinect open ni/ open cv */
	rc = HandViewerObj.Init(argc, argv);
	/* Check if init is ok? */
	if (rc != openni::STATUS_OK)
	{
		return 1;
	}
	
	/*Hand processing initialization. */
	//init_recording(&HandGestureSt);
	//init_windows();
	init_HandGestureSt(&HandGestureSt);

	handRecognition_Init();

	/*Inifinite loop to process hand */
	for(;;)
	{
		key = cvWaitKey(1);
		HandViewerObj.KeyBoard(key, NULL, NULL);
		
		if (HandViewerObj.training_flag == FALSE)
		{
			/*ReadImages Image */
			HandViewerObj.ReadImages();

			/*Dectect hand */
			HandSegmObj.HandSegmInit(HandViewerObj.pBiDepthImg, 
									 HandViewerObj.pDisplayImg, 
									 HandViewerObj.m_pHandTracker,
									 &HandViewerObj.handFrame);

			/*Hand dectection */
			HandSegmObj.HandDetection();

			//processing hands if hands are found
			//for (int idx = 0; idx <= HandSegmObj.GetNumHand(); idx++) {
				if(HandSegmObj.handFound[0])
				{
					//hand segmentation
					HandSegmObj.HandSegmentation(0);
					//HandSegm::~HandSegm();
					//hand feature extraction
					/*Map to hand gesture struct to process hand */
					HandGestureSt.dfdisthreshold = 5000/HandSegmObj.handPoint[0].d;
					HandGestureSt.HandPoint = cvPoint(HandSegmObj.handPoint[0].p.x, HandSegmObj.handPoint[0].p.y);
					HandGestureSt.image = HandViewerObj.pDisplayImg;
					HandGestureSt.thr_image = HandSegmObj.pThImg;
					//HandGestureSt.mm_image = this->pMMImg;
					HandGestureSt.handDepth = HandSegmObj.handPoint[0].d;
					HandGestureSt.RectTopHand = HandSegmObj.RectTop;
					//Call hand processing
					handProcessing();
					//Display Img
					HandViewerObj.DisPlayImg(HandSegmObj.pThImg);
					#ifdef GETSAMPLE
					{
						HandViewerObj.DisPlayImg(HandSegmObj.pThImg);
						cvWriteFrame(HandGestureSt.writer, HandSegmObj.img_t);//Write frame


						FILE*	depth_file = fopen("depthhand.txt", "w+" );

						fseek(depth_file, ftell(depth_file), SEEK_CUR);

						fprintf(depth_file, "%d,", HandSegmObj.handPoint[0].p.x);
						fprintf(depth_file, "%d,", HandSegmObj.handPoint[0].p.y);
						fprintf(depth_file, "%d,", HandSegmObj.handPoint[0].d);

						fclose(depth_file);
					}
					#else
					{
						//cvWriteFrame(HandGestureSt.writer, HandViewerObj.pDisplayImg);//Write frame
					}
					#endif
				}
				else// nohand found
				{
					HandViewerObj.DisPlayImg();
				}
			//}
			//HandSegmObj.ReleaseImg();
		}
		else
		{
			HandViewerObj.training_flag = FALSE;
			if(createDBC_s32(NULL) != 0)
			{
				printf("error in training data");
			}
		}
	}
}
