/*******************************************************************************
**                                                                            
**   File Name   : main.c
**   Project     : Hand Gesture Recognition Using Kinect 
**   Author      : Nguyen Trong Tri
**   Description : Header file for module hand viewer
**                                                                                                                                                   *
*******************************************************************************/

#ifndef __HAND_VIEWER_H__
#define	__HAND_VIEWER_H__

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/highgui/highgui_c.h>

#include "OpenNI.h"
#include "NiTE.h"
#include "HistoryBuffer.h"

#define WIDTH		  640
#define HEIGHT	      480

#define MAX_STR_NAME (ONI_MAX_STR>>4)

// Handviewer class
class HandViewer
{
public:
	HandViewer(const char* strSampleName, const char* strMWdName, const char* strThrWdName);
	 ~HandViewer();
	 
	openni::Status Init(int argc, char **argv); // OpenNI/Nite and OpenCV init
	void ReadImages();	//Does not return
	void DisPlayImg();	//Display Img
	void DisPlayImg(IplImage* pthrhImg);
	void KeyBoard(unsigned char key, int x, int y);

	nite::HandTracker*		m_pHandTracker;
	nite::HandTrackerFrameRef handFrame;
	IplImage* pBiDepthImg;
	IplImage* pDisplayImg;

	bool training_flag;
	
protected:
	//void DrawHistory(nite::HandTracker* pHandTracker, int id, HistoryBuffer<HISBUFFER>* pHistory);

private:
	/*********************************
	*	private methods
	*********************************/ 
	openni::Status InitOpenNI(void);
	openni::Status InitOpenCV(int argc, char **argv);
	void OnKey(unsigned char key, int x, int y);	
	HandViewer(const HandViewer&);
	HandViewer& operator=(HandViewer&);
	
	/*********************************
	* private variables
	*********************************/
	HandViewer* ms_self;	
	//int				m_pDepthHist[MAX_DEPTH];
	char			m_strSampleName[MAX_STR_NAME]; 
	char			m_MainImgName[MAX_STR_NAME];
	char			m_ThreImgName[MAX_STR_NAME];
	
	/*
	* OpenNI, NITE platform
	*/
	openni::Device			m_device;
	//openni::VideoStream		m_depthStream;
	openni::VideoStream		m_colorStream;
	/*
	* pointer points to images
	*/
//	IplImage* pDisplayImg;
	IplImage** ppDisplayImg;
	IplImage* pDepthImg;
	IplImage* pColorImg;
//	IplImage* pBiDepthImg;
	IplImage* p2BiDepthImg;
 	IplImage* pThImg;

	
	/* File recording handle */
	CvVideoWriter	*writer;	
};


#endif// __HAND_VIEWER_H__
