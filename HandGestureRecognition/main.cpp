/*******************************************************************************
*                                                                              *
*   PrimeSense NiTE 2.0 - Hand HandViewer Sample                                   *
*   Copyright (C) 2012 PrimeSense Ltd.                                         *
*                                                                              *
*******************************************************************************/

#include "HandViewer.h"

int main(int argc, char** argv)
{
	openni::Status rc = openni::STATUS_OK;
	HandViewer HandViewer("Hand Geture");

	rc = HandViewer.Init(argc, argv);
	if (rc != openni::STATUS_OK)
	{
		return 1;
	}

	for(;;)
	{
		HandViewer.Run();
	}
}
