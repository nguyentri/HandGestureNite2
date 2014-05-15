#include "PalmFinder.h"
#include "PointFunctions.h"

using namespace std;

extern HandGetureTypeSt  HandGestureSt;

PalmFinder::PalmFinder(HandGetureTypeSt *contour)
{
	InitializeInstanceFields();
	this->searchRadius = 4;
	this->contourReduction = 8;
	this->contour = contour;
	resultpointer = new Palm();
}


Palm* PalmFinder::FindCenter()
{
//C# TO C++ CONVERTER WARNING: C# to C++ Converter converted the original 'null' assignment to a call to 'delete', but you should review memory allocation of all CvPointer variables in the converted code:
	this->FindCenterFromCandidates();
	CvPoint location = this->resultpointer->getLocation();
//	this->IncreaseAccuracy(&location, contour);
	return resultpointer;
}

void PalmFinder::FindCenterFromCandidates()
{
	double* distances = new double[HandGestureSt.thresholdPoints.size()];

	memset(distances, 0, HandGestureSt.thresholdPoints.size());

	double result = DBL_MAX;
	CvPoint* contCvPoint;

	for(int index = 0; index < HandGestureSt.thresholdPoints.size(); index++)
	{
		result = DBL_MAX;
		for (int idx = 0; idx < HandGestureSt.contour->total; idx++)
		{
			contCvPoint = (CvPoint*)cvGetSeqElem(HandGestureSt.contour ,idx);
			if((HandGestureSt.thresholdPoints[index].x == contCvPoint->x) && (HandGestureSt.thresholdPoints[index].y == contCvPoint->y))
			{
				result = 0;
				break;
			}
			else
			{
				result = min(PointFunctions::Distance(*contCvPoint, HandGestureSt.thresholdPoints[index]), result);
			}
		}
		distances[index] = result;
	}

	double maxDistance = 0;
	int maxIndex = -1;
	for (int index = 0; index < HandGestureSt.thresholdPoints.size(); index++)
	{
		if (distances[index] > maxDistance)
		{
			maxDistance = distances[index];
			maxIndex = index;
		}
	}
	if (maxIndex >= 0)
	{
		this->resultpointer = new Palm(HandGestureSt.thresholdPoints[maxIndex], maxDistance);
	}
}

void PalmFinder::IncreaseAccuracy(CvPoint *center, HandGetureTypeSt *contour)
{
	auto newCandidateCvPoints = std::vector<CvPoint>();

	for (int x = -this->searchRadius; x <= this->searchRadius; x++)
	{
		for (int y = -this->searchRadius; y <= this->searchRadius; y++)
		{
			if (x != 0 && y != 0)
			{
				newCandidateCvPoints.push_back(cvPoint(center->x + x, center->y + y));
			}
		}
	}
//	this->FindCenterFromCandidates();
}

double PalmFinder::FindMaxDistance(CvPoint candidate)
{
	double result = DBL_MAX;
	CvPoint* contCvPoint;

	for (int idx = 0; idx < HandGestureSt.contour->total; idx++)
	{
		contCvPoint = (CvPoint*)cvGetSeqElem(HandGestureSt.contour ,idx);

		result = min(PointFunctions::Distance(*contCvPoint,  candidate), result);
	}
	return result;
}

void PalmFinder::InitializeInstanceFields()
{
	contourReduction = 0;
	searchRadius = 0;
}

