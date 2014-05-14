/*******************************************************************************
*                                                                              *
*   PrimeSense NiTE 2.0 - Hand Viewer Sample                                   *
*   Copyright (C) 2012 PrimeSense Ltd.                                         *
*                                                                              *
*******************************************************************************/

#include "Viewer.h"

int main(int argc, char** argv)
{
	openni::Status rc = openni::STATUS_OK;
	HandGesture HandGesture("Hand Geture");

	rc = HandGesture.Init(argc, argv);
	if (rc != openni::STATUS_OK)
	{
		return 1;
	}

	for(;;)
	{
		HandGesture.Run();

	}
}