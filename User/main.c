/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usbd_hid_core.h"
#include "usbd_usr.h"
#include "usbd_desc.h"

__ALIGN_BEGIN USB_OTG_CORE_HANDLE  USB_OTG_dev __ALIGN_END;

__IO uint32_t TimingDelay;
__IO uint8_t BeginFlag = 0x00;
__IO uint8_t UserButtonPressed = 0x00;

int16_t MPU6050_Offset[6];

MPU6050_Data MPU6050_Device_0_Data;
MPU6050_Data MPU6050_Device_1_Data;

// Private Functions
static uint32_t Demo_USBConfig(void);
static void TIM1_Config(void);
static void TIM2_Config(void);
static void TIM7_Config(void);
static void Encoder_Config(void);
static void Demo_Exec(void);
static void MPU6050_Config(MPU6050_Addr Device_ID);
static void EnableTimerInterrupt(uint8_t TIMx_IRQn, uint8_t Priority);

/*
Raw FIFO Data Format:
[ X''[n-2], X''[n-1], X''[n] ]
[ Y''[n-2], Y''[n-1], Y''[n] ]
[ Z''[n-2], Z''[n-1], Z''[n] ]
[ Yaw'[n-2], Yaw'[n-1], Yaw'[n] ]
[ Pitch'[n-2], Pitch'[n-1], Pitch'[n] ]
[ Roll'[n-2], Roll'[n-1], Roll'[n] ]
*/
float MPU6050_Data_FIFO[6][3]={0};

/*
Integrated_Data_FIFO Format:
[ X'[n-2], X'[n-1], X'[n] ]
[ Y'[n-2], Y'[n-1], Y'[n] ]
[ Z'[n-2], Z'[n-1], Z'[n] ]
*/
float Velocity_Data_FIFO [3][3]={0};

//Stores the final Rotation and Displacement data
float Rotation_Data[3]={0};
float Displacement_Data[3]={0};

/*
Main Function
*/
int main(void){
  printf("Hello World\n");
	
  //initalize user button
  STM_EVAL_PBInit(BUTTON_USER, BUTTON_MODE_EXTI); 
  
	Demo_Exec();
}

static void EnableTimerInterrupt(uint8_t TIMx_IRQn, uint8_t Priority){
	  NVIC_InitTypeDef nvicStructure;
    nvicStructure.NVIC_IRQChannel = TIMx_IRQn;
    nvicStructure.NVIC_IRQChannelPreemptionPriority = 0;
    nvicStructure.NVIC_IRQChannelSubPriority = Priority;
    nvicStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvicStructure);
}


//Execute the demo application, code from STM32F4 demo package using it for a base.
static void Demo_Exec(void){
  RCC_ClocksTypeDef RCC_Clocks;
  uint8_t togglecounter = 0x00;
	
  /* SysTick end of count event each 1ms */
  RCC_GetClocksFreq(&RCC_Clocks);
  SysTick_Config(RCC_Clocks.HCLK_Frequency / 1000);  
  while(1)
  {
    BeginFlag = 0x00;
    
    /* Reset UserButton_Pressed variable, when the EXTI0_IRQHandler is called by pushing the push button UserButtonPressed is set to 0x01 */
    UserButtonPressed = 0x00;
    
    /* Initialize LEDs to be managed by GPIO */
    STM_EVAL_LEDInit(LED3);
    STM_EVAL_LEDInit(LED4);
    STM_EVAL_LEDInit(LED5);
    STM_EVAL_LEDInit(LED6);

    /* Turn OFF all LEDs */
    STM_EVAL_LEDOff(LED3);
    STM_EVAL_LEDOff(LED4);
    STM_EVAL_LEDOff(LED5);
    STM_EVAL_LEDOff(LED6);
    
    /* Waiting while user button is not pressed */
    while (UserButtonPressed == 0x00)
    {
      togglecounter ++;
      if (togglecounter == 0x10)
      {
        togglecounter = 0x00;
        while (togglecounter < 0x10)
        {
          STM_EVAL_LEDToggle(LED4);
          STM_EVAL_LEDToggle(LED3);
          STM_EVAL_LEDToggle(LED5);
          STM_EVAL_LEDToggle(LED6);
          DelayMs(500);
          togglecounter ++;
        }
       togglecounter = 0x00;
      }
    }
    
    /* Waiting until user button is released to clear the user button pressed variable*/
    while (STM_EVAL_PBGetState(BUTTON_USER) == Bit_SET)
    {}
    UserButtonPressed = 0x00;
    
		/* Initialize MPU-6050 */
		MPU6050_Config(MPU6050_Device_0);
		
		printf("MPU6050 Initialization Complete, Offsets Acquired. Beginning Interrupt Config... \n");
    /* TIM channels configuration IRQ is disabled while configuration is in progress.*/
		__disable_irq();
    TIM7_Config();

    
    Demo_USBConfig();
    __enable_irq();
    /* Waiting User Button is pressed */
    while (UserButtonPressed == 0x00)
    {}
    
    /* Waiting User Button is Released */
    while (STM_EVAL_PBGetState(BUTTON_USER) == Bit_SET)
    {}
			
    /* Disconnect the USB device */
    DCD_DevDisconnect(&USB_OTG_dev);
    USB_OTG_StopDevice(&USB_OTG_dev);
  }
}



//Initializes the USB for the demonstration application.
static uint32_t Demo_USBConfig(void){
  USBD_Init(&USB_OTG_dev,
            USB_OTG_FS_CORE_ID,
            &USR_desc, 
            &USBD_HID_cb, 
            &USR_cb);
  
  return 0;
}

//Configures the MPU6050, uses I2C and pins B8 B9
static void MPU6050_Config(MPU6050_Addr Device_ID){
	if(MPU6050_Init(I2C1, Device_ID,	MPU6050_Accel_4G,	MPU6050_Gyro_250ds)!=MPU6050_OK)
	{
		Fail_Handler();
	}
	if (MPU6050_Read(I2C1,MPU6050_Device_0,MPU6050_INT_STATUS,0)){
		MPU6050_ReadAll(I2C1,&MPU6050_Device_0_Data,MPU6050_Device_0);
		MPU6050_Offset[0]=MPU6050_Device_0_Data.Accel_X;
		MPU6050_Offset[1]=MPU6050_Device_0_Data.Accel_Y;
		MPU6050_Offset[2]=MPU6050_Device_0_Data.Accel_Z;
		MPU6050_Offset[3]=MPU6050_Device_0_Data.Gyro_X;
		MPU6050_Offset[4]=MPU6050_Device_0_Data.Gyro_Y;
		MPU6050_Offset[5]=MPU6050_Device_0_Data.Gyro_Z;
	}
}


//Configures the TIM1 Peripheral, this is used for PWM to the H bridges, uses E9 E11 E13
static void TIM1_Config(void){
	GPIO_InitTypeDef GPIO_InitStruct;
	TIM_OCInitTypeDef TIM_OCInitStruct;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct;
	
	//Turn on the HCLK for TIM1 and GPIO A
	RCC_APB1PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	
	//Using PD10 and PD11 for PWM to HBridges
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_11 | GPIO_Pin_13;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_Init(GPIOE, &GPIO_InitStruct);
	
	/* Connect TIM4 pins to AF2 */  
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource9, GPIO_AF_TIM1);
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_TIM1); 
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource13, GPIO_AF_TIM1); 
	
	/* Time base configuration */
	
	//Period max value is 0xFFFF/65535, NOTE VALUE ONLY MATTERS FOR PWM MODE, IN TOGGLE OR OC MODE SET IT TO 0!
	
	TIM_TimeBaseStruct.TIM_Prescaler = 2-1;
  TIM_TimeBaseStruct.TIM_Period = 13125-1; //6.4KHz clock
  
  TIM_TimeBaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStruct);
	
	TIM_ARRPreloadConfig(TIM1, ENABLE);
	
	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1; 
	TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable; 
	TIM_OCInitStruct.TIM_OutputNState = TIM_OutputNState_Enable; 
	TIM_OCInitStruct.TIM_Pulse = 50; //pulse is the CCR register in OC mode
	TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High; 
	TIM_OCInitStruct.TIM_OCNPolarity = TIM_OCNPolarity_High; //OCN is the output of the output compare register, by setting OCN polarity to high we allow the output to follow the OC when it is high.
	TIM_OCInitStruct.TIM_OCIdleState = TIM_OCIdleState_Set; 
	TIM_OCInitStruct.TIM_OCNIdleState = TIM_OCIdleState_Reset; 
	
	TIM_OC1Init(TIM1, &TIM_OCInitStruct);
	TIM_OC2Init(TIM1, &TIM_OCInitStruct);
	TIM_OC3Init(TIM1, &TIM_OCInitStruct);
	
  TIM_CCxCmd(TIM1, TIM_Channel_1, ENABLE);
  TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);

  TIM_CCxCmd(TIM1, TIM_Channel_2, ENABLE);
  TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
	
	TIM_CCxCmd(TIM1, TIM_Channel_3, ENABLE);
  TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);

  TIM_Cmd(TIM1, ENABLE);
	
	TIM_ITConfig(TIM1, TIM_IT_CC1, ENABLE); 
	EnableTimerInterrupt(TIM1_CC_IRQn, 0);
}

//Hall effect interface uses pins A1 A2 A3 A15, Enable output uses C0 C1 C4
static void TIM2_Config(void){ 
	GPIO_InitTypeDef GPIO_InitStructure;
  TIM_TimeBaseInitTypeDef TIM_InitStructure;
  TIM_ICInitTypeDef TIM_ICInitStructure;
	
	//output to EN1 EN2 EN3 on L6234
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	RCC_APB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	
  // inputs for hall sensors use PIN 15 for reset signal
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  RCC_APB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM2);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_TIM2);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_TIM2);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource15, GPIO_AF_TIM2);
	
	// enable clock for timer2
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
  
  // update event every ca. 4295sec
  // resolution: 1usec
  TIM_InitStructure.TIM_Prescaler = 42;
  TIM_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_InitStructure.TIM_Period = 0xFFFFFFFF;
  TIM_InitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  
  TIM_TimeBaseInit(TIM2, &TIM_InitStructure);
  // connect three channels to XOR in timer
  TIM_SelectHallSensor(TIM2, ENABLE);
  
  TIM_SelectInputTrigger(TIM2, TIM_TS_TI1F_ED);
  TIM_SelectSlaveMode(TIM2, TIM_SlaveMode_Reset);
  
  // initialize the cature compare function of timer2
  TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
  TIM_ICInitStructure.TIM_ICFilter = 0xF;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_TRC;
  
  TIM_ICInit(TIM2, &TIM_ICInitStructure); 
  
   // enable the interrupt for timer2
  TIM_ITConfig(TIM2, TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3 , ENABLE);
  
  EnableTimerInterrupt(TIM2_IRQn, 0);
  
   // enable timer2
  TIM_Cmd(TIM2, ENABLE);
}


//MPU6050 Interrupt config, checks MPU6050 at 10KHz to ensure newest data is obtained.
static void TIM7_Config(void){
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStruct;
  
  /* TIM4 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);
  
  /* Time base configuration */
	TIM_TimeBaseStruct.TIM_Prescaler = 1680;//Gives us a 100KHz Clock
  TIM_TimeBaseStruct.TIM_Period = 10;//Gives us one interrupt every 10 counts, 10KHz
  TIM_TimeBaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStruct);
	TIM_Cmd(TIM7, ENABLE);
  TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);
	EnableTimerInterrupt(TIM7_IRQn, 0);
}
//configure the quadrature encoder reading uses B6 B7 A0 TIM4
static void Encoder_Config(void){
	GPIO_InitTypeDef GPIO_InitStruct;
	EXTI_InitTypeDef EXTI_InitStruct;
  NVIC_InitTypeDef NVIC_InitStruct;
	
	RCC_AHB1PeriphClockCmd (RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd (RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd (RCC_APB1Periph_TIM4, ENABLE);
	RCC_APB2PeriphClockCmd (RCC_APB2Periph_SYSCFG, ENABLE);
	
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;//Selecting AF mode allows connection of pin to PWM output	
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_TIM4); 

	TIM_EncoderInterfaceConfig (TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
	TIM_SetAutoreload (TIM4, 2000);//number of encoder signals in one rotation of motor = 4*500 = 2000;
	TIM_Cmd(TIM4,ENABLE);
	
	//Reset signal for the encoder is on the same line as the blue button
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource0);
  EXTI_InitStruct.EXTI_Line = EXTI_Line0;
  EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling;  //use falling edge since port is pulled up.
  EXTI_InitStruct.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStruct);

  NVIC_InitStruct.NVIC_IRQChannel = EXTI0_IRQn ;
  NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x05;
  NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x05;
  NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStruct); 
}



//Process data is called by TIM7 IRQ.
TODO: reduce number of axis being processed, we only need one axis of the gyro, reduce number of axis being read as well.
void Process_Data(void){
	float temp;
	uint8_t n;
	__disable_irq();//not sure if its a good idea to disable interrupts
	if (MPU6050_Read(I2C1,MPU6050_Device_0,MPU6050_INT_STATUS,0)){
		for(n=0;n<6;n++)
		{
			//Shift FIFO buffer to the left by one unit, the data at [n][0] is removed and a new data will be added later at [n][2]
			MPU6050_Data_FIFO[n][0]=MPU6050_Data_FIFO[n][1];
			MPU6050_Data_FIFO[n][1]=MPU6050_Data_FIFO[n][2];
		}
		for(n=0;n<3;n++)
		{
			//Shift FIFO buffer to the left by one unit, the data at [n][0] is removed and a new data will be added later at [n][2]
			Velocity_Data_FIFO[n][0]=Velocity_Data_FIFO[n][1];
			Velocity_Data_FIFO[n][1]=Velocity_Data_FIFO[n][2];
		}
		
		//Read Normally
		MPU6050_ReadAll(I2C1,&MPU6050_Device_0_Data,MPU6050_Device_0);
		
		//EXTREMELY processor intensive takes about 100 CPU cycles... not sure if better method is available...
		MPU6050_Data_FIFO[0][2]=(float)MPU6050_Device_0_Data.Accel_X/MPU6050_ACCEL_SENS_4;
		MPU6050_Data_FIFO[1][2]=(float)MPU6050_Device_0_Data.Accel_Y/MPU6050_ACCEL_SENS_4;
		MPU6050_Data_FIFO[2][2]=(float)MPU6050_Device_0_Data.Accel_Z/MPU6050_ACCEL_SENS_4;
		
		MPU6050_Data_FIFO[3][2]=(float)MPU6050_Device_0_Data.Gyro_X/MPU6050_GYRO_SENS_250;
		MPU6050_Data_FIFO[4][2]=(float)MPU6050_Device_0_Data.Gyro_Y/MPU6050_GYRO_SENS_250;
		MPU6050_Data_FIFO[5][2]=(float)MPU6050_Device_0_Data.Gyro_Z/MPU6050_GYRO_SENS_250;
		
		//Velocity Data - 20 Cycles each?
		temp=(MPU6050_Data_FIFO[0][0]+4*MPU6050_Data_FIFO[0][1]+MPU6050_Data_FIFO[0][2]);
		Velocity_Data_FIFO[0][2]=Velocity_Data_FIFO[0][2]+0.0005f*temp;
		
		temp=(MPU6050_Data_FIFO[1][0]+4*MPU6050_Data_FIFO[1][1]+MPU6050_Data_FIFO[0][2]);
		Velocity_Data_FIFO[1][2]=Velocity_Data_FIFO[1][2]+0.0005f*temp;
		
		temp=(MPU6050_Data_FIFO[2][0]+4*MPU6050_Data_FIFO[2][1]+MPU6050_Data_FIFO[2][2]);
		Velocity_Data_FIFO[2][2]=Velocity_Data_FIFO[2][2]+0.0005f*temp;
		
		//Rotation Data
		temp=(MPU6050_Data_FIFO[3][0]+4*MPU6050_Data_FIFO[3][1]+MPU6050_Data_FIFO[0][2]);
		Rotation_Data[0]=Rotation_Data[0]+0.0005f*temp;
		
		temp=(MPU6050_Data_FIFO[4][0]+4*MPU6050_Data_FIFO[4][1]+MPU6050_Data_FIFO[0][2]);
		Rotation_Data[0]=Rotation_Data[0]+0.0005f*temp;
		
		temp=(MPU6050_Data_FIFO[5][0]+4*MPU6050_Data_FIFO[5][1]+MPU6050_Data_FIFO[2][2]);
		Rotation_Data[0]=Rotation_Data[0]+0.0005f*temp;
		
		//Displacement Data
		temp=(Velocity_Data_FIFO[0][0]+4*Velocity_Data_FIFO[0][1]+Velocity_Data_FIFO[0][2]);
		Displacement_Data[0]=Displacement_Data[0]+0.0005f*temp;
		
		temp=(Velocity_Data_FIFO[1][0]+4*Velocity_Data_FIFO[1][1]+Velocity_Data_FIFO[0][2]);
		Displacement_Data[1]=Displacement_Data[1]+0.0005f*temp;
		
		temp=(Velocity_Data_FIFO[2][0]+4*Velocity_Data_FIFO[2][1]+Velocity_Data_FIFO[2][2]);
		Displacement_Data[2]=Displacement_Data[2]+0.0005f*temp;
	}
	__enable_irq();
	TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
}


//nTime: specifies the DelayMs time length, in 10 ms.
void DelayMs(__IO uint32_t nTime){
  TimingDelay = nTime;

  while(TimingDelay != 0);
}



//Decrements the TimingDelay variable.
void TimingDelay_Decrement(void){
  if (TimingDelay != 0x00)
  { 
    TimingDelay--;
  }
}



//This function handles the test program fail.
void Fail_Handler(void){
  while(1)
  {
    /* Toggle Red LED */
    STM_EVAL_LEDToggle(LED5);
    DelayMs(250);
  }
}
