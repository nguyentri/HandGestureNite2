
#include <conio.h>
#include <stdio.h>
#include "HandLabel.h"
#include "HandMoment.h"
#include "Utilities.h"

const char* fingerName[] = {"little", "ring", "middle" ,"index", "thumb"};

const char* numberLIST[] = {"0", "1", "2", "3", "4", "5", "6", "7", "8", "9", "10"};

void nameFingers(HandGetureTypeSt* pHandGestureSt)
  /* Use the finger tip coordinates, and the contour's COG and axis angle to horizontal
     to label the fingers.

     Try to label the thumb and index based on their likely angle ranges
     relative to the COG. This assumes that the thumb and index finger are on the
     left side of the hand.

     Then label the other fingers based on the order of the names in the FingerName class
  */
{ // reset all named fingers to unknown

	std::vector<FingerNameE>namedFingers = pHandGestureSt->namedFingers;

    namedFingers.clear();
    for (int i=0; i < pHandGestureSt->num_fingers; i++)
		namedFingers.push_back(UNKNOWN);

    labelFingers(pHandGestureSt);

    // printFingers("named fingers", namedFingers);
    //labelUnknowns(namedFingers);
    // printFingers("revised named fingers", namedFingers);
}  // end of nameFingers()



void labelFingers(HandGetureTypeSt* pHandGestureSt)
  // attempt to label the thumb and index fingers of the hand
{ 
    bool foundThumb = false;
    bool foundIndex = false;
	bool foundMiddle = false;
	bool foundRing = false;
	bool foundLittle = false;
	float angleThr = 0;

      /* the thumb and index fingers will most likely be stored at the end
         of the list, since the contour hull was built in a counter-clockwise 
         order by the call to cvConvexHull2() in findFingerTips(), and I am assuming
         the thumb is on the left of the hand.
         So iterate backwards through the list.
      */
	float contourAxisAngle = pHandGestureSt->contourAxisAngle;
	if(pHandGestureSt->num_fingers >= 1)
	{
		int i = pHandGestureSt->num_fingers - 1;
		while ((i >= 0)) {
			int angle = angleToCOG(pHandGestureSt->fingers[i], pHandGestureSt->hand_center, pHandGestureSt->contourAxisAngle);


		  // check for thumb
		  if ((angle <=  MAX_THUMB) && (angle > MIN_THUMB) && !foundThumb) {
			cvPutText(pHandGestureSt->image, "Thumb", cvPointMove(pHandGestureSt->fingers[i], pHandGestureSt->RectTopHand), &pHandGestureSt->cvFontFingerName, PURPLE);
			  pHandGestureSt->namedFingers.assign(i, THUMB);
			foundThumb = true;
		  }

		  // check for index
		  if ((angle <= MAX_INDEX) && (angle > MIN_INDEX) && !foundIndex) {
			pHandGestureSt->namedFingers.assign(i, INDEX);
			cvPutText(pHandGestureSt->image, "Index", cvPointMove(pHandGestureSt->fingers[i], pHandGestureSt->RectTopHand), &pHandGestureSt->cvFontFingerName, PURPLE);
			foundIndex = true;
		  }

		  // check for middle
		  if ((angle <= MAX_MIDDLE) && (angle > MIN_MIDDLE) && !foundMiddle) {
			pHandGestureSt->namedFingers.assign(i, MIDDLE);
			cvPutText(pHandGestureSt->image, "Middle",cvPointMove(pHandGestureSt->fingers[i], pHandGestureSt->RectTopHand), &pHandGestureSt->cvFontFingerName, PURPLE);
			foundMiddle = true;
		  }

		  // check for ring
		  if ((angle <= MAX_RING) && (angle > MIN_RING) && !foundRing) {
			pHandGestureSt->namedFingers.assign(i, RING);
			cvPutText(pHandGestureSt->image, "Ring", cvPointMove(pHandGestureSt->fingers[i], pHandGestureSt->RectTopHand), &pHandGestureSt->cvFontFingerName, PURPLE);
			foundRing = true;
		  }

		  // check for little
		  if ((angle <= MAX_LITTLE) && (angle > MIN_LITTLE) && !foundLittle) {
			pHandGestureSt->namedFingers.assign(i, MIDDLE);
			cvPutText(pHandGestureSt->image, "Little", cvPointMove(pHandGestureSt->fingers[i], pHandGestureSt->RectTopHand), &pHandGestureSt->cvFontFingerName, PURPLE);
			foundLittle = true;
		  }

		  i--;
		}
	}


	//switch (pHandGestureSt->num_fingers)
	//{
	//	case 0:
	//		strcpy(pHandGestureSt->number, numberLIST[0]);
	//	break;
	//	case 1:
	//		if(foundIndex == true)
	//			strcpy(pHandGestureSt->number, numberLIST[1]);
	//		else if(foundThumb == true)
	//			strcpy(pHandGestureSt->number, numberLIST[10]);
	//		else 
	//		{
	//			strcpy(pHandGestureSt->number, "N.A");
	//		}
	//	break;
	//	case 2:
	//		strcpy(pHandGestureSt->number, numberLIST[2]);
	//	break;
	//	case 3:
	//		if(foundThumb && foundIndex && foundMiddle)
	//		{
	//			strcpy(pHandGestureSt->number, numberLIST[3]);
	//		}
	//		else if(foundIndex && foundMiddle && foundRing)
	//		{
	//			strcpy(pHandGestureSt->number, numberLIST[6]);
	//		}
	//		else if(foundLittle && foundMiddle && foundIndex)
	//		{
	//			strcpy(pHandGestureSt->number, numberLIST[7]);
	//		}
	//		else if(foundLittle && foundRing && foundIndex)
	//		{
	//			strcpy(pHandGestureSt->number, numberLIST[8]);
	//		}
	//		else if(foundLittle && foundRing && foundMiddle)
	//		{
	//			strcpy(pHandGestureSt->number, numberLIST[9]);
	//		}
	//		else 
	//		{
	//			strcpy(pHandGestureSt->number, "N.A");
	//		}
	//	break;
	//	case 4:
	//		strcpy(pHandGestureSt->number, numberLIST[4]);
	//	break;
	//	case 5:
	//		strcpy(pHandGestureSt->number, numberLIST[5]);
	//	break;
	//	default:
	//		strcpy(pHandGestureSt->number, numberLIST[0]);
	//	break;
	//}
}  // end of labelFingers()

