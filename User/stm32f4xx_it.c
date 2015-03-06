/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    19-September-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides all exceptions handler and peripherals interrupt
  *          service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_it.h"
#include "main.h"
#include "usb_core.h"
#include "usbd_core.h"
#include "stm32f4xx.h"
#include "usbd_hid_core.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define CURSOR_STEP     3
#define USIGNED_INT_MAX 65535
#define SIGNED_INT_MAX 32767
#define PHASE_OFFSET_120 667
#define PHASE_OFFSET_240 1333

const unsigned short int Hall2En[8][3]={{0,0,0},{0,1,1},{1,1,0},{1,0,1},{1,0,1},{1,1,0},{0,1,1},{0,0,0}};
	
//NOTE: PWM data only covers 0-180 degrees, the other 180 degrees is assumed to be 0 as output.
const unsigned char PWMdata[1000]={0,1,1,1,2,2,2,3,3,3,3,4,4,4,5,5,5,6,6,6,7,7,7,8,8,8,8,9,9,9,10,10,10,11,11,11,12,12,12,13,13,13,13,14,14,14,
	15,15,15,16,16,16,17,17,17,18,18,18,18,19,19,19,20,20,20,21,21,21,22,22,22,22,23,23,23,24,24,24,25,25,25,25,26,26,26,27,27,27,28,28,28,29,29,
	29,29,30,30,30,31,31,31,31,32,32,32,33,33,33,34,34,34,34,35,35,35,36,36,36,37,37,37,37,38,38,38,39,39,39,39,40,40,40,41,41,41,41,42,42,42,43,
	43,43,43,44,44,44,45,45,45,45,46,46,46,47,47,47,47,48,48,48,48,49,49,49,50,50,50,50,51,51,51,51,52,52,52,53,53,53,53,54,54,54,54,55,55,55,55,
	56,56,56,56,57,57,57,58,58,58,58,59,59,59,59,60,60,60,60,61,61,61,61,62,62,62,62,63,63,63,63,64,64,64,64,64,65,65,65,65,66,66,66,66,67,67,67,
	67,68,68,68,68,68,69,69,69,69,70,70,70,70,70,71,71,71,71,72,72,72,72,72,73,73,73,73,74,74,74,74,74,75,75,75,75,75,76,76,76,76,76,77,77,77,77,
	77,78,78,78,78,78,79,79,79,79,79,80,80,80,80,80,81,81,81,81,81,81,82,82,82,82,82,83,83,83,83,83,83,84,84,84,84,84,84,85,85,85,85,85,85,86,86,
	86,86,86,86,87,87,87,87,87,87,87,88,88,88,88,88,88,89,89,89,89,89,89,89,90,90,90,90,90,90,90,90,91,91,91,91,91,91,91,92,92,92,92,92,92,92,92,
	93,93,93,93,93,93,93,93,93,94,94,94,94,94,94,94,94,94,95,95,95,95,95,95,95,95,95,95,95,96,96,96,96,96,96,96,96,96,96,96,97,97,97,97,97,97,97,
	97,97,97,97,97,97,98,98,98,98,98,98,98,98,98,98,98,98,98,98,98,98,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,100,
	100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,
	100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,99,99,99,99,99,99,99,99,99,99,99,
	99,99,99,99,99,99,99,99,99,99,99,99,99,98,98,98,98,98,98,98,98,98,98,98,98,98,98,98,98,97,97,97,97,97,97,97,97,97,97,97,97,97,96,96,96,96,96,
	96,96,96,96,96,96,95,95,95,95,95,95,95,95,95,95,95,94,94,94,94,94,94,94,94,94,93,93,93,93,93,93,93,93,93,92,92,92,92,92,92,92,92,91,91,91,91,
	91,91,91,90,90,90,90,90,90,90,90,89,89,89,89,89,89,89,88,88,88,88,88,88,87,87,87,87,87,87,87,86,86,86,86,86,86,85,85,85,85,85,85,84,84,84,84,
	84,84,83,83,83,83,83,83,82,82,82,82,82,81,81,81,81,81,81,80,80,80,80,80,79,79,79,79,79,78,78,78,78,78,77,77,77,77,77,76,76,76,76,76,75,75,75,
	75,75,74,74,74,74,74,73,73,73,73,72,72,72,72,72,71,71,71,71,70,70,70,70,70,69,69,69,69,68,68,68,68,68,67,67,67,67,66,66,66,66,65,65,65,65,64,
	64,64,64,64,63,63,63,63,62,62,62,62,61,61,61,61,60,60,60,60,59,59,59,59,58,58,58,58,57,57,57,56,56,56,56,55,55,55,55,54,54,54,54,53,53,53,53,
	52,52,52,51,51,51,51,50,50,50,50,49,49,49,48,48,48,48,47,47,47,47,46,46,46,45,45,45,45,44,44,44,43,43,43,43,42,42,42,41,41,41,41,40,40,40,39,
	39,39,39,38,38,38,37,37,37,37,36,36,36,35,35,35,34,34,34,34,33,33,33,32,32,32,31,31,31,31,30,30,30,29,29,29,29,28,28,28,27,27,27,26,26,26,25,
	25,25,25,24,24,24,23,23,23,22,22,22,22,21,21,21,20,20,20,19,19,19,18,18,18,18,17,17,17,16,16,16,15,15,15,14,14,14,13,13,13,13,12,12,12,11,11,
	11,10,10,10,9,9,9,8,8,8,8,7,7,7,6,6,6,5,5,5,4,4,4,3,3,3,3,2,2,2,1,1,1,0,0}; 



//phase index for the three phases used to get correct PWM data, if index is >1000 then return 0, if index is >2000 index=0
//every MPU6050 read cycle gives us a new value for theta, at the same time we should read the value of the encoder to get an value for error,
//using the value for error we can get angular velocity/frequency then using that we can get the sin value output->P1Index++; PWMdata[f*P1Index], this is checked every main pwm out cycle;
unsigned short int P1Index=0,P2Index=667,P3Index=1333;

extern uint8_t BeginFlag;
extern uint8_t UserButtonPressed;
extern float Velocity_Data_FIFO [3][3];

/* Private function prototypes -----------------------------------------------*/
extern USB_OTG_CORE_HANDLE           USB_OTG_dev;
static uint8_t *USBD_HID_GetPos (void);
extern uint32_t USBD_OTG_ISR_Handler (USB_OTG_CORE_HANDLE *pdev);

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
  uint8_t *buf;
  
  if (BeginFlag == 0x00)
  {
    TimingDelay_Decrement();
  }
  else
  {
    buf = USBD_HID_GetPos();
    if((buf[1] != 0) ||(buf[2] != 0))
    {
      USBD_HID_SendReport (&USB_OTG_dev, 
                           buf,
                           4);
    } 
  }
  
}

/******************************************************************************/
/*                 STM32Fxxx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32fxxx.s).                                               */
/******************************************************************************/
//IRQ handler for each PWM pulse
void TIM1_CC_IRQHandler(void){
	if (TIM_GetITStatus(TIM1, TIM_IT_CC1) != RESET){
		
		TIM_ClearITPendingBit(TIM1, TIM_IT_CC1);
	}
	else{
		TIM_ClearITPendingBit(TIM1, TIM_IT_CC1);
	}
}
//IRQ handler for each hall effect signal
void TIM2_CC_IRQHandler(void){
	unsigned char temp;
	if (TIM_GetITStatus(TIM2, TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3) != RESET){
		/*
		Once a new hall state is defined we check if it is the same as the old state (to remove errors), 
		if its a new state then we update by reading all 3 IC channels to get a 3 bit value (use tim function),
		from the 3 bit value and direction of travel we can set the polarity for each channel using an array that stores the correct configuration
		
		The correct configuration can be found by using an oscilioscope that is attached to each phase to 
		read the corresponding backemf output and hall sensor feedback after manually turning the motor
		*/
		temp=4*TIM_GetCapture1(TIM2)+2*TIM_GetCapture2(TIM2)+TIM_GetCapture3(TIM2);
		if(temp>9)
		{
			temp=0;
		}
		
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
		
		TIM_ClearITPendingBit(TIM2, TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3);
	}
	else{
		TIM_ClearITPendingBit(TIM2, TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3);
	}
}



//IRQ handler for each MPU6050 update (10KHz)
void TIM7_IRQHandler(void){
	if (TIM_GetITStatus(TIM7, TIM_IT_Update) != RESET){
	  if (MPU6050_Read(I2C1,MPU6050_Device_0,MPU6050_INT_STATUS,0)){
				
				//TODO: using found direction set the duty cycle accordingly
				/*if(dir)
				{
					setDuty(approxsin());
				}
				else
				{
					setDuty(approxsin());
				}*/
			
				Process_Data();//Note IT pending bit will be cleared AFTER processing is complete.
			}
    }
		else{
			TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
		}
}


//use one interrupt to handle both starting code and encoder reset signal.
void EXTI0_IRQHandler(void){
  UserButtonPressed = 0x01;
  TIM_SetCounter(TIM2, 0);
  /* Clear the EXTI line pending bit */
  EXTI_ClearITPendingBit(USER_BUTTON_EXTI_LINE);
}

/**
  * @brief  This function handles EXTI15_10_IRQ Handler.
  * @param  None
  * @retval None
  */
void OTG_FS_WKUP_IRQHandler(void)
{
  if(USB_OTG_dev.cfg.low_power)
  {
	/* Reset SLEEPDEEP and SLEEPONEXIT bits */
	SCB->SCR &= (uint32_t)~((uint32_t)(SCB_SCR_SLEEPDEEP_Msk | SCB_SCR_SLEEPONEXIT_Msk));

	/* After wake-up from sleep mode, reconfigure the system clock */
	SystemInit();
    USB_OTG_UngateClock(&USB_OTG_dev);
  }
  EXTI_ClearITPendingBit(EXTI_Line18);
}

/**
  * @brief  This function handles OTG_HS Handler.
  * @param  None
  * @retval None
  */
void OTG_FS_IRQHandler(void)
{
  USBD_OTG_ISR_Handler (&USB_OTG_dev);
}


//Send Position for USB mouse function
static uint8_t *USBD_HID_GetPos (void){
  static uint8_t HID_Buffer[4] = {0};
  
  HID_Buffer[1] = 0;
  HID_Buffer[2] = 0;
  /* LEFT Direction */
  if((Velocity_Data_FIFO[1][2]) < -2)
  {
    HID_Buffer[1] += CURSOR_STEP;
  }
  /* RIGHT Direction */ 
  if((Velocity_Data_FIFO[1][2]) > 2)
  {
   HID_Buffer[1] -= CURSOR_STEP;
  } 
  /* UP Direction */
  if((Velocity_Data_FIFO[2][2]) < -2)
  {
    HID_Buffer[2] += CURSOR_STEP;
  }
  /* DOWN Direction */ 
  if((Velocity_Data_FIFO[2][2]) > 2)
  {
    HID_Buffer[2] -= CURSOR_STEP;
  } 
  
  return HID_Buffer;
}

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/