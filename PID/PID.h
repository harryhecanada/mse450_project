#ifndef __PID_H
#define __PID_H
//Includes

//Relevant Structures
typedef struct {
	float KP;
	float KD;
	float KI;
	float c[3];
	float Ts;
	float upplim;//should be 200
	float lowlim;//should be 1
	float InputBuffer[3];	
	float OutputBuffer[2];
	
} PID_TypeDef;

//Function Prototypes
float CalcPID_Out(PID_TypeDef PIDin, float err);
void PID_Init(PID_TypeDef PIDin);
#endif
