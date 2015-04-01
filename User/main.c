/* Includes ------------------------------------------------------------------*/
#include "main.h"

int16_t MPU6050_Offset[6];

MPU6050_Data MPU6050_Device_0_Data;

//defines the number of counts in one clock period, one clock period being 42MHz/MAIN_CLOCK_PERIOD_COUNT = 2KHz
#define MAIN_CLOCK_PERIOD_COUNT 21000-1

// Private Functions
static void TIM1_Config(void);
static void TIM2_Config(void);
static void TIM7_Config(void);
static void Encoder_Config(void);
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
	//__disable_irq();
	 
	//Gyro & Accel Configuration
	MPU6050_Config(MPU6050_Device_0);
	SysTick_Config(SystemCoreClock / 50); 
	//TIM7_Config();
	//Main PWM Configuration
	TIM1_Config();
	//Hall Interface
	TIM2_Config();
	//Encoder Configuration
	//Encoder_Config();
	//__enable_irq();
	while(1)
	{
	}
}

static void EnableTimerInterrupt(uint8_t TIMx_IRQn, uint8_t Priority){
	  NVIC_InitTypeDef nvicStructure;
    nvicStructure.NVIC_IRQChannel = TIMx_IRQn;
    nvicStructure.NVIC_IRQChannelPreemptionPriority = Priority;
    nvicStructure.NVIC_IRQChannelSubPriority = Priority;
    nvicStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvicStructure);
}

//Configures the MPU6050, uses I2C and pins B8 (SCL) B9 (SDA)
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


//Configures the TIM1 Peripheral, this is used for PWM to the H bridges, uses E9 E11 E13, 
//the interrupt on TIM1 also dictates the sampling frequency of the encoder and the overall system
static void TIM1_Config(void){
	GPIO_InitTypeDef GPIO_InitStruct;
	TIM_OCInitTypeDef TIM_OCInitStruct;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct;
	
	//Turn on the HCLK for TIM1 and GPIO A
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	
	//Using PD10 and PD11 for PWM to HBridges
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_11 | GPIO_Pin_13;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;//GPIO_PuPd_NOPULL
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_Init(GPIOE, &GPIO_InitStruct);
	
	/* Connect TIM4 pins to AF2 */  
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource9, GPIO_AF_TIM1);
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_TIM1); 
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource13, GPIO_AF_TIM1); 
	
	/* Time base configuration */
	//Period max value is 0xFFFF/65535, NOTE VALUE ONLY MATTERS FOR PWM MODE, IN TOGGLE OR OC MODE SET IT TO 0!
	
	TIM_TimeBaseStruct.TIM_Prescaler = 4-1;//84MHz/4=21000KHz
  TIM_TimeBaseStruct.TIM_Period = 21000;//1KHz
  TIM_TimeBaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStruct);

	
	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM2; 
	TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;  
	TIM_OCInitStruct.TIM_Pulse = 1000; //pulse is the CCR register in OC mode
	TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_Low; 
	//TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_Low;
  //TIM_OCInitStruct.TIM_OCIdleState = TIM_OCIdleState_Set;
	
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
	TIM_ARRPreloadConfig(TIM1, ENABLE);	
	TIM_CtrlPWMOutputs(TIM1, ENABLE);


	
	TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE); 
	EnableTimerInterrupt(TIM1_UP_TIM10_IRQn, 0);
}

//Hall effect interface uses pins A1 A2 A3, Enable output uses C0 C1 C4
static void TIM2_Config(void){ 
	GPIO_InitTypeDef GPIO_InitStructure;
  TIM_TimeBaseInitTypeDef TIM_InitStructure;
  TIM_ICInitTypeDef TIM_ICInitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	
	//output to EN1 EN2 EN3 on L6234
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
  // inputs for hall sensors
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM2);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_TIM2);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_TIM2);
	
  // resolution: 1usec
  TIM_InitStructure.TIM_Prescaler = 84-1;
  TIM_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_InitStructure.TIM_Period = 60000;
  TIM_InitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  
  TIM_TimeBaseInit(TIM2, &TIM_InitStructure);
  // connect three channels to XOR in timer
  //TIM_SelectHallSensor(TIM2, ENABLE);
  
  //TIM_SelectInputTrigger(TIM2, TIM_TS_ETRF);//this needs to be changed for 3 channel non xor, not sure to what.
  //TIM_SelectSlaveMode(TIM2, TIM_SlaveMode_External1);
  
  // initialize the cature compare function of timer2
  TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
  TIM_ICInitStructure.TIM_ICFilter = 0xF; //Could be 0x0, doesnt make big difference
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge; //Use both edges
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1; //no prescaler
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //Connect Channel to Trigger
  
  TIM_ICInit(TIM2, &TIM_ICInitStructure); 
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_3;
	TIM_ICInit(TIM2, &TIM_ICInitStructure); 
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_4;
	TIM_ICInit(TIM2, &TIM_ICInitStructure); 
	
	
  EnableTimerInterrupt(TIM2_IRQn, 0);
   // enable the interrupt for timer2
  TIM_ITConfig(TIM2, TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4 , ENABLE);
   // enable timer2
  TIM_Cmd(TIM2, ENABLE);
}


//MPU6050 Interrupt config, checks MPU6050 at 10KHz to ensure newest data is obtained.
static void TIM7_Config(void){
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStruct;
  
  /* TIM4 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);//42MHz CLK, 2x for Timers -> 84MHz
  
  /* Time base configuration */
	TIM_TimeBaseStruct.TIM_Prescaler = 840-1;//Gives us a 100KHz Clock
  TIM_TimeBaseStruct.TIM_Period = 10000-1;//Gives us one interrupt every 1000 counts, 1KHz
  TIM_TimeBaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStruct);
	TIM_Cmd(TIM7, ENABLE);
  TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);
	EnableTimerInterrupt(TIM7_IRQn, 2);
}
//configure the quadrature encoder reading uses B6 B7 TIM4
static void Encoder_Config(void){
	GPIO_InitTypeDef GPIO_InitStruct;
	
	RCC_AHB1PeriphClockCmd (RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd (RCC_APB1Periph_TIM4, ENABLE);
	
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;//Selecting AF mode allows connection of pin to PWM output	
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_TIM4); 

	TIM_EncoderInterfaceConfig (TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
	TIM_SetAutoreload (TIM4, 2000);//number of encoder signals in one rotation of motor = 4*500 = 2000;
	TIM_Cmd(TIM4,ENABLE);
	//to read counter use: int reading=TIM_GetCounter(TIM4);
}
//Process data is called by TIM7 IRQ.
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
		/*for(n=0;n<3;n++)
		{
			//Shift FIFO buffer to the left by one unit, the data at [n][0] is removed and a new data will be added later at [n][2]
			Velocity_Data_FIFO[n][0]=Velocity_Data_FIFO[n][1];
			Velocity_Data_FIFO[n][1]=Velocity_Data_FIFO[n][2];
		}*/
		
		//Read Normally
		MPU6050_ReadAll(I2C1,&MPU6050_Device_0_Data,MPU6050_Device_0);
		
		//EXTREMELY processor intensive takes about 100 CPU cycles... not sure if better method is available...
		//MPU6050_Data_FIFO[0][2]=(float)MPU6050_Device_0_Data.Accel_X/MPU6050_ACCEL_SENS_4;
		//MPU6050_Data_FIFO[1][2]=(float)MPU6050_Device_0_Data.Accel_Y/MPU6050_ACCEL_SENS_4;
		//MPU6050_Data_FIFO[2][2]=(float)MPU6050_Device_0_Data.Accel_Z/MPU6050_ACCEL_SENS_4;
		
		//MPU6050_Data_FIFO[3][2]=(float)MPU6050_Device_0_Data.Gyro_X/MPU6050_GYRO_SENS_250;
		//MPU6050_Data_FIFO[4][2]=(float)MPU6050_Device_0_Data.Gyro_Y/MPU6050_GYRO_SENS_250;
		//printf("Gyro Z = %d \n", MPU6050_Device_0_Data.Gyro_Z);
		if(MPU6050_Device_0_Data.Gyro_Z>100 || MPU6050_Device_0_Data.Gyro_Z<-500)
		{
			MPU6050_Data_FIFO[5][2]=(float)MPU6050_Device_0_Data.Gyro_Z/MPU6050_GYRO_SENS_250;
			//printf("Gyro Z = %f \n", MPU6050_Data_FIFO[5][2]);
		}
		/*Velocity Data - 20 Cycles each?
		temp=(MPU6050_Data_FIFO[0][0]+4*MPU6050_Data_FIFO[0][1]+MPU6050_Data_FIFO[0][2]);
		Velocity_Data_FIFO[0][2]=Velocity_Data_FIFO[0][2]+0.0005f*temp;
		
		temp=(MPU6050_Data_FIFO[1][0]+4*MPU6050_Data_FIFO[1][1]+MPU6050_Data_FIFO[0][2]);
		Velocity_Data_FIFO[1][2]=Velocity_Data_FIFO[1][2]+0.0005f*temp;
		
		temp=(MPU6050_Data_FIFO[2][0]+4*MPU6050_Data_FIFO[2][1]+MPU6050_Data_FIFO[2][2]);
		Velocity_Data_FIFO[2][2]=Velocity_Data_FIFO[2][2]+0.0005f*temp;
		*/
		
		//Rotation Data
	//	temp=(MPU6050_Data_FIFO[3][0]+4*MPU6050_Data_FIFO[3][1]+MPU6050_Data_FIFO[0][2]);
		//Rotation_Data[0]=Rotation_Data[0]+0.0005f*temp;
		
		//temp=(MPU6050_Data_FIFO[4][0]+4*MPU6050_Data_FIFO[4][1]+MPU6050_Data_FIFO[0][2]);
		//Rotation_Data[1]=Rotation_Data[1]+0.0005f*temp;
		
		temp=(MPU6050_Data_FIFO[5][0]+4*MPU6050_Data_FIFO[5][1]+MPU6050_Data_FIFO[2][2]);
		Rotation_Data[2]=Rotation_Data[2]+0.0005f*temp;
		
		/*Displacement Data
		temp=(Velocity_Data_FIFO[0][0]+4*Velocity_Data_FIFO[0][1]+Velocity_Data_FIFO[0][2]);
		Displacement_Data[0]=Displacement_Data[0]+0.0005f*temp;
		
		temp=(Velocity_Data_FIFO[1][0]+4*Velocity_Data_FIFO[1][1]+Velocity_Data_FIFO[0][2]);
		Displacement_Data[1]=Displacement_Data[1]+0.0005f*temp;
		
		temp=(Velocity_Data_FIFO[2][0]+4*Velocity_Data_FIFO[2][1]+Velocity_Data_FIFO[2][2]);
		Displacement_Data[2]=Displacement_Data[2]+0.0005f*temp;
		*/
	}
	//Print out readings from gyro when updating data
	//printf("Gyro Z = %f \n", Rotation_Data[2]);
	__enable_irq();
	//TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
}

//This function handles the test program fail.
void Fail_Handler(void){
	printf("Critical Failure \n");
  while(1)
	{
  }
}
