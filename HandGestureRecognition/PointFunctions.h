#pragma once

#include <cmath>
#include <opencv2/core/core.hpp>

class PointFunctions
{
public:
	static double Distance(CvPoint p1, CvPoint p2);

	static double Distance(float x1, float y1, float x2, float y2);

	static CvPoint Center(CvPoint p1, CvPoint p2);

};
