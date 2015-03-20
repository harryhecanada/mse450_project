/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "stm32f4_MPU6050.h"
#include <stdio.h>

#define ABS(x)         (x < 0) ? (-x) : x
#define MAX(a,b)       (a < b) ? (b) : a
/* Exported functions ------------------------------------------------------- */
void Process_Data(void);
void Fail_Handler(void);
