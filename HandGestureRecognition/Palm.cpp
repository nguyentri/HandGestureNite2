
#include "Palm.h"


Palm::Palm(CvPoint location, double distanceToContour)
{
	InitializeInstanceFields();
	this->location = location;
	this->distanceToContour = distanceToContour;
}

CvPoint Palm::getLocation() const
{
	return this->location;
}

const double Palm::getDistanceToContour() const
{
	return this->distanceToContour;
}

void Palm::InitializeInstanceFields()
{
	distanceToContour = 0;
}
