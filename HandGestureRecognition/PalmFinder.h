#pragma once

#include "Palm.h"
#include <vector>
#include <float.h>


class PalmFinder
{
	private:
		Palm* resultpointer;

		float contourReduction;
		int searchRadius;

		HandGetureTypeSt* contour;

	public:
		PalmFinder(HandGetureTypeSt *contour);

		Palm* FindCenter();

	private:
		void FindCenterFromCandidates();

		void IncreaseAccuracy(CvPoint *center, HandGetureTypeSt *contour);

		double FindMaxDistance(CvPoint candidate);

	private:
		void InitializeInstanceFields();
};

