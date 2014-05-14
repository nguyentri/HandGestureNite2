#include "PointFunctions.h"

double PointFunctions::Distance(CvPoint p1, CvPoint p2)
{
	return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
}

double PointFunctions::Distance(float x1, float y1, float x2, float y2)
{
	return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
}

CvPoint PointFunctions::Center(CvPoint p1, CvPoint p2)
{
	CvPoint point = cvPoint((p1.x + p2.x) / 2, (p1.y + p2.y)/2);
	return  point;
}
