/*
PID Controller
*/

//Includes
#include "PID.h"
#include <stdio.h>

float CalcPID_Out(PID_TypeDef* PIDin, float input, float feedback){
	float result=0;
	float err=input-feedback;
	int temp=0;
	//PID calculations, note that compiler only can handle one floating point multiplication or divison per line->
	PIDin->InputBuffer[0]=err;
	
	result += PIDin->OutputBuffer[1];
	result += PIDin->c[0]*PIDin->InputBuffer[0];
	result += PIDin->c[1]*PIDin->InputBuffer[1];
	result += PIDin->c[2]*PIDin->InputBuffer[2];
	
	//Shift buffers
	PIDin->OutputBuffer[1]=PIDin->OutputBuffer[0];
	PIDin->OutputBuffer[0]=result;
	PIDin->InputBuffer[2]=PIDin->InputBuffer[1];
	PIDin->InputBuffer[1]=PIDin->InputBuffer[0];
	
	//Saturation
	if(input<0)
	{
		temp=-1;
	}
	else
	{
		temp=1;
	}
	if(temp*PIDin->OutputBuffer[0] <= PIDin->lowlim){
		PIDin->OutputBuffer[0] = temp*PIDin->lowlim;
	}else if(temp*PIDin->OutputBuffer[0] >= PIDin->upplim){
		PIDin->OutputBuffer[0] = temp*PIDin->upplim;
	}
	return PIDin->OutputBuffer[0];
}

void PID_Init(PID_TypeDef* PIDin){
	//Calculate discrete time domain controller constants
	PIDin->c[0] = PIDin->KP + PIDin->KD + PIDin->KI;
	PIDin->c[1] = -1*(PIDin->KP + 2*PIDin->KD);
	PIDin->c[2] = PIDin->KD;
}



