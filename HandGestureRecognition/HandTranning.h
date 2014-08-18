#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/highgui/highgui_c.h>

void saveTrainingImage(const IplImage*	input_image)
{
   static int imgfpsIdx = 0;
   static int imgtrainingIdx = 0;
   
   char ImgFileName_str[20] =".\\images\\imgxxxx.jpg";
   /* Get file name. */
   sprintf(str_temp,"%04d", imgtrainingIdx);
   strncpy(ImgFileName_str + 13, str_temp, 4);
  
	if((imgIdx % 60) == 0)
	{
		cvSaveImage(ImgFileName_str, pHandGestureSt->thr_image);
		imgtrainingIdx++;
	}

	if(imgtrainingIdx == 9000)
	{
		imgtrainingIdx = 0;
	}
	imgIdx++;
}
