/*******************************************************************************
*                                                                              *
*   PrimeSense NiTE 2.0 - Hand HandViewer Sample                                   *
*   Copyright (C) 2012 PrimeSense Ltd.                                         *
*                                                                              *
*******************************************************************************/

#include "HandViewer.h"
#include "HandSegm.h"
//#include "GestureRecognition.h"

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
	
	/*Inifinite loop to process hand */
	for(;;)
	{
		key = cvWaitKey(1);
		HandViewerObj.KeyBoard(key, NULL, NULL);

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
				HandViewerObj.DisPlayImg(HandSegmObj.pThImg);
			}
			else// nohand found
			{
				HandViewerObj.DisPlayImg();
			}
		//}
		HandSegmObj.ReleaseImg();
	}
}
