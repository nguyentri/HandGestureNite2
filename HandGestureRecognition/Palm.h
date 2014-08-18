
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp> 
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/highgui/highgui_c.h>
#include <HandFeatEx.h>

class Palm
{
	private:
		CvPoint location;
		double distanceToContour;

	public:
		Palm(CvPoint location, double distanceToContour);

		Palm(){distanceToContour = 0;};

		CvPoint getLocation() const;

		const double getDistanceToContour() const;

	private:
		void InitializeInstanceFields();
};

