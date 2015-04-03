/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_it.h"
#include "main.h"
#include "stm32f4xx.h"

/* Private define ------------------------------------------------------------*/
#define DUTY_MULT 3

//Converts Hall signal to correct Enable pin combinations on the L6234 driver for commutation
const unsigned short int Hall2En[8][3]={{0,0,0},{0,1,1},{1,1,0},{1,0,1},{1,0,1},{1,1,0},{0,1,1},{0,0,0}};
//PWMdata, approximates Sin function from 0 to 180 degrees (0 to 1000) and also provides 0 from 1000 to 2000 to reduce current drain
const unsigned char PWMdata[2000]={0,1,1,1,2,2,2,3,3,3,3,4,4,4,5,5,5,6,6,6,7,7,7,8,8,8,8,9,9,9,10,10,10,11,11,11,12,12,12,13,
	13,13,13,14,14,14,15,15,15,16,16,16,17,17,17,18,18,18,18,19,19,19,20,20,20,21,21,21,22,22,22,22,23,23,23,24,24,24,25,25,25,
	25,26,26,26,27,27,27,28,28,28,29,29,29,29,30,30,30,31,31,31,31,32,32,32,33,33,33,34,34,34,34,35,35,35,36,36,36,37,37,37,37,
	38,38,38,39,39,39,39,40,40,40,41,41,41,41,42,42,42,43,43,43,43,44,44,44,45,45,45,45,46,46,46,47,47,47,47,48,48,48,48,49,49,
	49,50,50,50,50,51,51,51,51,52,52,52,53,53,53,53,54,54,54,54,55,55,55,55,56,56,56,56,57,57,57,58,58,58,58,59,59,59,59,60,60,
	60,60,61,61,61,61,62,62,62,62,63,63,63,63,64,64,64,64,64,65,65,65,65,66,66,66,66,67,67,67,67,68,68,68,68,68,69,69,69,69,70,
	70,70,70,70,71,71,71,71,72,72,72,72,72,73,73,73,73,74,74,74,74,74,75,75,75,75,75,76,76,76,76,76,77,77,77,77,77,78,78,78,78,
	78,79,79,79,79,79,80,80,80,80,80,81,81,81,81,81,81,82,82,82,82,82,83,83,83,83,83,83,84,84,84,84,84,84,85,85,85,85,85,85,86,
	86,86,86,86,86,87,87,87,87,87,87,87,88,88,88,88,88,88,89,89,89,89,89,89,89,90,90,90,90,90,90,90,90,91,91,91,91,91,91,91,92,
	92,92,92,92,92,92,92,93,93,93,93,93,93,93,93,93,94,94,94,94,94,94,94,94,94,95,95,95,95,95,95,95,95,95,95,95,96,96,96,96,96,
	96,96,96,96,96,96,97,97,97,97,97,97,97,97,97,97,97,97,97,98,98,98,98,98,98,98,98,98,98,98,98,98,98,98,98,99,99,99,99,99,99,
	99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,
	100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,
	100,100,100,100,100,100,100,100,100,100,100,100,100,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,98,
	98,98,98,98,98,98,98,98,98,98,98,98,98,98,98,97,97,97,97,97,97,97,97,97,97,97,97,97,96,96,96,96,96,96,96,96,96,96,96,95,95,95,
	95,95,95,95,95,95,95,95,94,94,94,94,94,94,94,94,94,93,93,93,93,93,93,93,93,93,92,92,92,92,92,92,92,92,91,91,91,91,91,91,91,90,
	90,90,90,90,90,90,90,89,89,89,89,89,89,89,88,88,88,88,88,88,87,87,87,87,87,87,87,86,86,86,86,86,86,85,85,85,85,85,85,84,84,84,
	84,84,84,83,83,83,83,83,83,82,82,82,82,82,81,81,81,81,81,81,80,80,80,80,80,79,79,79,79,79,78,78,78,78,78,77,77,77,77,77,76,76,
	76,76,76,75,75,75,75,75,74,74,74,74,74,73,73,73,73,72,72,72,72,72,71,71,71,71,70,70,70,70,70,69,69,69,69,68,68,68,68,68,67,67,
	67,67,66,66,66,66,65,65,65,65,64,64,64,64,64,63,63,63,63,62,62,62,62,61,61,61,61,60,60,60,60,59,59,59,59,58,58,58,58,57,57,57,
	56,56,56,56,55,55,55,55,54,54,54,54,53,53,53,53,52,52,52,51,51,51,51,50,50,50,50,49,49,49,48,48,48,48,47,47,47,47,46,46,46,45,
	45,45,45,44,44,44,43,43,43,43,42,42,42,41,41,41,41,40,40,40,39,39,39,39,38,38,38,37,37,37,37,36,36,36,35,35,35,34,34,34,34,33,
	33,33,32,32,32,31,31,31,31,30,30,30,29,29,29,29,28,28,28,27,27,27,26,26,26,25,25,25,25,24,24,24,23,23,23,22,22,22,22,21,21,21,
	20,20,20,19,19,19,18,18,18,18,17,17,17,16,16,16,15,15,15,14,14,14,13,13,13,13,12,12,12,11,11,11,10,10,10,9,9,9,8,8,8,8,7,7,7,6,
	6,6,5,5,5,4,4,4,3,3,3,3,2,2,2,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; 

//Initial offsets for each phase's index
unsigned short int P1Index=0,P2Index=667,P3Index=1333;

//Obtain require data structures from main
extern float MPU6050_Data_FIFO[6][3];
extern float Velocity_Data_FIFO [3][3];
extern float Rotation_Data[3];
extern PID_TypeDef PIDin;

//Values used to to calculate feedback velocity and output velocity
float Encoder0=0;
float Encoder1=0;
int vel=0;

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
	//Process MPU6050 Data
	Process_Data();
	
	//Debug printfs
	//printf("Encoder = %d\n", TIM_GetCounter(TIM4));
	//printf("P1 = %d ", P1Index);
	//printf("P2 = %d ", P2Index);
	//printf("P3 = %d \n", P3Index);
	//printf("Gyro Z = %f \n", Rotation_Data[2]);
}
/******************************************************************************/
/*                 STM32Fxxx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32fxxx.s).                                               */
/******************************************************************************/
//IRQ handler for each PWM pulse
void TIM1_UP_TIM10_IRQHandler(void){
	if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET){
		
		static int deadtime=0;
		static int dir=0;
		static int oEnc=0;
		int temp=0;
		
		//Prevent the situation where the encoder jumps from 2000 to 0 or 0 to 2000.
		temp=TIM_GetCounter(TIM4);
		if((oEnc-temp)>1500 || (oEnc+temp)>1500)
		{
			oEnc=temp;
			temp=1;
			
		}
		else
		{
			oEnc=temp;
			temp=0;
		}
		//only update velocity if output of encoder is "normal"
		//Converts encoder readings to angular velocity in rad/s
		if(temp)
		{
			Encoder1=Encoder0;
			Encoder0=(float)oEnc*0.00314159265f;
		}
		else
		{
			Encoder1=Encoder0;
			Encoder0=(float)oEnc*0.00314159265f;
			
			//call PID update to update output, currently set to constant angular velocity of 40 rad/s
			vel=(int)CalcPID_Out(PIDin,40-(Encoder0-Encoder1)/0.001f);
			//vel=(int)CalcPID_Out(PIDin,Rotation_Data[2]-(Encoder0-Encoder1)/0.001f);
			
			//conversion from phase velocity to angular velocity
			vel=vel/DUTY_MULT/4*3;
		}
		
		//Convert velocity + direction into deadtime, commutation direction, and speed.
		if(vel<0)
		{
			temp=-1;
			vel*=-1;
		}
		else
		{
			temp=1;
		}
		if (temp>0 && dir>0)
		{
			dir=0;
			deadtime=1000;
			temp=P3Index;
			P3Index=P2Index;
			P2Index=temp;
		}
		else if(temp<0 && dir<1)
		{
			dir=1;
			deadtime=1000;
			temp=P3Index;
			P3Index=P2Index;
			P2Index=temp;
		}
		
		//Normal Operation
		if(deadtime<1)
		{
			//Increment phase velocity counters
			P1Index+=vel;
			P2Index+=vel;
			P3Index+=vel; 

			//set CCR values
			TIM_SetCompare1(TIM1,vel*DUTY_MULT*PWMdata[P1Index]);
			TIM_SetCompare2(TIM1,vel*DUTY_MULT*PWMdata[P2Index]);
			TIM_SetCompare3(TIM1,vel*DUTY_MULT*PWMdata[P3Index]);
			
			//Roll back phase index values if they exceed PWMdata's range
			if(P1Index>2000)
			{
				P1Index-=2000;
			}
			if(P2Index>2000)
			{
				P2Index-=2000;
			}
			if(P3Index>2000)
			{
				P3Index-=2000;
			}
		}
		else
		{
			//Dead time wait, so we dont go over current.
			deadtime--;
			TIM_SetCompare1(TIM1,0);
			TIM_SetCompare2(TIM1,0);
			TIM_SetCompare3(TIM1,0);
		}
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
	}
	else{
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
	}
}
//IRQ handler for each hall effect signal
void TIM2_IRQHandler(void){
	uint8_t temp;
	if (TIM_GetITStatus(TIM2, TIM_IT_CC2 | TIM_IT_CC3 | TIM_IT_CC4) != RESET){
		
		//Convert the reading into int
		temp=4*GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_1);
		temp+=2*GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_2);
		temp+=GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_3);
		
		//Process based on the commutation sequence
		if(Hall2En[temp][0])
		{
			GPIO_SetBits(GPIOC, GPIO_Pin_0);
		}
		else
		{
			GPIO_ResetBits(GPIOC, GPIO_Pin_0);
		}
		
		if(Hall2En[temp][1])
		{
			GPIO_SetBits(GPIOC, GPIO_Pin_1);
		}
		else
		{
			GPIO_ResetBits(GPIOC, GPIO_Pin_1);
		}
		
		if(Hall2En[temp][2])
		{
			GPIO_SetBits(GPIOC, GPIO_Pin_4);
		}
		else
		{
			GPIO_ResetBits(GPIOC, GPIO_Pin_4);
		}
		TIM_ClearITPendingBit(TIM2, TIM_IT_CC2 | TIM_IT_CC3 | TIM_IT_CC4);
	}
	else{
		TIM_ClearITPendingBit(TIM2, TIM_IT_CC2 | TIM_IT_CC3 | TIM_IT_CC4);
	}
	
}




