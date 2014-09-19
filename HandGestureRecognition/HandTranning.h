
#ifndef HAND_TRAINING_H
#define HAND_TRAINING_H

#ifdef HAND_TRAINING
	#define EXTERN
#else
	#define EXTERN extern
#endif


#define FINGER_NUM 		5
#define SAMPLE_NUM_MAX  32
#define DEPTH_NUM		1
#define GEST_NUM		11

extern const char* trDBC_FileName_c;

typedef struct TRAININGDATA_ST
{
	uint8_t finger_num_u8;
	float angle_f[FINGER_NUM];
	float dis_f[FINGER_NUM];
	float angleDefect_f[4];
};


EXTERN	TRAININGDATA_ST trainingData_st;

void sortArray_V(float* const arr_pc, const uint8_t arrLen_u8);

int createDBC_s32(const IplImage*	input_image);


#endif