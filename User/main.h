/**
  ******************************************************************************
  * @file    main.h 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    19-September-2011
  * @brief   Header for main.c module
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
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F4_DISCOVERY_DEMO_H
#define __STM32F4_DISCOVERY_DEMO_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4_discovery.h"
#include "stm32f4_MPU6050.h"
//#include <stdio.h>

#define ABS(x)         (x < 0) ? (-x) : x
#define MAX(a,b)       (a < b) ? (b) : a

#define CURSOR_STEP 3
/* Exported functions ------------------------------------------------------- */
short int approxsin(int angle);
void Process_Data(void);
void DelayMs(__IO uint32_t nTime);
void TimingDelay_Decrement(void);
void Fail_Handler(void);
#endif /* __STM32F4_DISCOVERY_DEMO_H */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
