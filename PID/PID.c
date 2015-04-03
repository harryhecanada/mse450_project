/*
PID Controller
*/

//Includes
#include "PID.h"
#include <stdio.h>



float CalcPID_Out(PID_TypeDef PIDin, float err){
	float result;
	int temp=0;
	result += PIDin.OutputBuffer[1];
	result += PIDin.c[0]*PIDin.InputBuffer[0];
	result += PIDin.c[1]*PIDin.InputBuffer[1];
	result += PIDin.c[2]*PIDin.InputBuffer[2];
	
	PIDin.OutputBuffer[1]=PIDin.OutputBuffer[0];
	PIDin.OutputBuffer[0]=result;
	PIDin.InputBuffer[2]=PIDin.InputBuffer[1];
	PIDin.InputBuffer[1]=PIDin.InputBuffer[0];
	PIDin.InputBuffer[0]=err;
	//Saturation
	if(result<0)
	{
		temp=-1;
	}
	else
	{
		temp=1;
	}
	if(temp*PIDin.OutputBuffer[0] <= PIDin.lowlim){
		PIDin.OutputBuffer[0] = temp*PIDin.lowlim;
	}else if(temp*PIDin.OutputBuffer[0] >= PIDin.upplim){
		PIDin.OutputBuffer[0] = temp*PIDin.upplim;
	}
	return PIDin.OutputBuffer[0];
	
}

void PID_Init(PID_TypeDef PIDin){
	

	PIDin.c[0] = PIDin.KP + PIDin.KD + PIDin.KI;
	PIDin.c[1] = -1*(PIDin.KP + 2*PIDin.KD);
	PIDin.c[2] = PIDin.KD;
	
}



