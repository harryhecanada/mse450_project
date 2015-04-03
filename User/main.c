/*
 ******************************************************************************
  * @project 3 Phase Speed Control Using MPU6050 as Velocity Input
  * @author  Harry He, Mavin Bautista, Kevin Mathiasen
  * @version V1.0
  * @date    April 3, 2015
  * @brief   The Main C file initializes all the required modules for the project.
  *         
 ******************************************************************************
*/
#include "main.h"

//Global structures are used to pass initalized data structures to interrupts
MPU6050_Data MPU6050_Device_0_Data;
PID_TypeDef PIDin;

// PID definitions
#define PID_KD  0.0141
#define PID_KP  0.00591
#define PID_KI  0.000099302
#define PID_TS  1/1000;

// Private Functions
static void PID_Config(void);
static void TIM1_Config(void);
static void TIM2_Config(void);
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
	//Send test message
  printf("Hello World\n");
	
	//PID configuration
	PID_Config();
	
	//Gyro & Accel Configuration
	MPU6050_Config(MPU6050_Device_0);
	//Systick is used to poll the gyro and also used for debugging.
	SysTick_Config(SystemCoreClock / 1000); 
	
	//Main PWM Configuration
	TIM1_Config();
	
	//Hall Interface
	TIM2_Config();
	
	//Encoder Configuration
	Encoder_Config();
	while(1)
	{
	}
}

//Setup PID
static void PID_Config()
{
	PIDin.KD=PID_KD;
	PIDin.KI=PID_KI;
	PIDin.KP=PID_KP;
	PIDin.Ts=PID_TS;
	PIDin.lowlim=1;
	PIDin.upplim=200;
	PID_Init(PIDin);
}

//Function used to simplify NVIC calls, All Interrupt Handlers are in stm32f4xx_it.c
static void EnableTimerInterrupt(uint8_t TIMx_IRQn, uint8_t Priority){
	  NVIC_InitTypeDef nvicStructure;
    nvicStructure.NVIC_IRQChannel = TIMx_IRQn;
    nvicStructure.NVIC_IRQChannelPreemptionPriority = Priority;
    nvicStructure.NVIC_IRQChannelSubPriority = Priority;
    nvicStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvicStructure);
}

//Configures the MPU6050, uses I2C1 and pins B8 (SCL) B9 (SDA)
static void MPU6050_Config(MPU6050_Addr Device_ID){

	if(MPU6050_Init(I2C1, Device_ID,	MPU6050_Accel_4G,	MPU6050_Gyro_250ds)!=MPU6050_OK)
	{
		Fail_Handler();
	}
}


/*
Configures the TIM1 Peripheral, this is used for PWM to the H bridges, uses E9 E11 E13,
the interrupt on TIM1 also dictates the sampling frequency of the PID control.
*/
static void TIM1_Config(void){
	GPIO_InitTypeDef GPIO_InitStruct;
	TIM_OCInitTypeDef TIM_OCInitStruct;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct;
	
	//Turn on the HCLK for TIM1 and GPIO E
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	
	//Using PE9, PE11, PE13 for PWM to HBridges
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_11 | GPIO_Pin_13;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;//GPIO_PuPd_NOPULL
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_Init(GPIOE, &GPIO_InitStruct);
	//AF selection
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource9, GPIO_AF_TIM1);
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_TIM1); 
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource13, GPIO_AF_TIM1); 
	
	/* Time base configuration */
	//AHB2 Timer clock is 84MHz by default, prescaler divides down to 21MHz, period divides down to 1KHz
	TIM_TimeBaseStruct.TIM_Prescaler = 4-1;
  TIM_TimeBaseStruct.TIM_Period = 21000-1;
  TIM_TimeBaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStruct);

	//PWM mode 2 turns off when value is CCR is reached, PWM mode 1 turns on when CCR is reached.
	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM2; 
	TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;  
	TIM_OCInitStruct.TIM_Pulse = 1000; //pulse is the CCR register in OC mode
	TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_Low; 
	
	TIM_OC1Init(TIM1, &TIM_OCInitStruct);
	TIM_OC2Init(TIM1, &TIM_OCInitStruct);
	TIM_OC3Init(TIM1, &TIM_OCInitStruct);
	
  TIM_CCxCmd(TIM1, TIM_Channel_1, ENABLE);
  TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);

  TIM_CCxCmd(TIM1, TIM_Channel_2, ENABLE);
  TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
	
	TIM_CCxCmd(TIM1, TIM_Channel_3, ENABLE);
  TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
	
	//Enable TIM1 interrupts
	TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE); 
	EnableTimerInterrupt(TIM1_UP_TIM10_IRQn, 0);
	
	//Turn ON timer
	TIM_Cmd(TIM1, ENABLE);
	TIM_ARRPreloadConfig(TIM1, ENABLE);	
	
	//Turn ON PWM
	TIM_CtrlPWMOutputs(TIM1, ENABLE);
}

//Hall effect interface uses pins A1 A2 A3, Enable output uses C0 C1 C4
static void TIM2_Config(void){ 
	GPIO_InitTypeDef GPIO_InitStructure;
  TIM_TimeBaseInitTypeDef TIM_InitStructure;
  TIM_ICInitTypeDef TIM_ICInitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	
	//output to EN1 EN2 EN3 on L6234, digital output with pull up resistor
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
  //inputs for hall sensors, requires pull up resistors.
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM2);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_TIM2);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_TIM2);
	
  //resolution: 84MHz/84=1MHz=1us
  TIM_InitStructure.TIM_Prescaler = 84-1;
  TIM_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_InitStructure.TIM_Period = 50000;
  TIM_InitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseInit(TIM2, &TIM_InitStructure);
 
  //initialize input capture of TIM 2 CH2, CH3 and CH4
  TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
  TIM_ICInitStructure.TIM_ICFilter = 0xF; //Low pass filter could be 0x0, doesnt make big difference
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge; //Use both edges
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1; //no prescaler
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //Direct input... indirect is weird...
  
  TIM_ICInit(TIM2, &TIM_ICInitStructure); 
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_3;
	TIM_ICInit(TIM2, &TIM_ICInitStructure); 
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_4;
	TIM_ICInit(TIM2, &TIM_ICInitStructure); 
	
	// enable the interrupt for timer2 
  TIM_ITConfig(TIM2, TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4 , ENABLE);
	EnableTimerInterrupt(TIM2_IRQn, 0);
	
  // enable timer2
  TIM_Cmd(TIM2, ENABLE); 
}
//configure the quadrature encoder reading uses B6 B7 TIM4
static void Encoder_Config(void){
	GPIO_InitTypeDef GPIO_InitStruct;
	
	//Turn on GPIOB and TIM4
	RCC_AHB1PeriphClockCmd (RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd (RCC_APB1Periph_TIM4, ENABLE);
	
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;//Selecting AF mode allows connection of pin to PWM output	
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;//Encoder has built in pull up resistors
	GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_TIM4); 
	
	//Turn on encoder interface, built in function
	TIM_EncoderInterfaceConfig (TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
	
	//number of encoder signals in one rotation of motor = 4*500 = 2000;
	TIM_SetAutoreload (TIM4, 2000);
	
	//Enable TIM4
	TIM_Cmd(TIM4,ENABLE);
	//to read counter use: int reading=TIM_GetCounter(TIM4);
}
//Process data is called by TIM7 IRQ.
void Process_Data(void){
	float temp;
	//gyroscopes have drift, has to be reduced with drift compensator
	static float drift=0;
	
	//Disable interrupts to better handle processing
	__disable_irq();
	//Check if data is ready
	if (MPU6050_Read(I2C1,MPU6050_Device_0,MPU6050_INT_STATUS,0)){
		//Read from gyroscope
		MPU6050_ReadAll(I2C1,&MPU6050_Device_0_Data,MPU6050_Device_0);
		//Calculate Drift
		drift=MPU6050_Data_FIFO[5][2]-MPU6050_Data_FIFO[5][1];
		//Shift FIFO buffer to the left, originally 6 axis scraped due to processing speed limits.
		MPU6050_Data_FIFO[5][0]=MPU6050_Data_FIFO[5][1];
		MPU6050_Data_FIFO[5][1]=MPU6050_Data_FIFO[5][2];
		
		//If drift is small, ignore it (else), if its big then change rotation count.
		if(drift>3 || drift<-3)
		{
			MPU6050_Data_FIFO[5][2]=(float)MPU6050_Device_0_Data.Gyro_Z/MPU6050_GYRO_SENS_250;
			temp=(MPU6050_Data_FIFO[5][0]+4*MPU6050_Data_FIFO[5][1]+MPU6050_Data_FIFO[2][2]);
			Rotation_Data[2]=Rotation_Data[2]+0.0005f*temp;
		}
		else
		{
			MPU6050_Data_FIFO[5][2]=(float)MPU6050_Device_0_Data.Gyro_Z/MPU6050_GYRO_SENS_250;
		}
		/*
			for(n=0;n<3;n++)
			{
				//Shift FIFO buffer to the left by one unit, the data at [n][0] is removed and a new data will be added later at [n][2]
				Velocity_Data_FIFO[n][0]=Velocity_Data_FIFO[n][1];
				Velocity_Data_FIFO[n][1]=Velocity_Data_FIFO[n][2];
			}
			//3 Axis Accel Processing
			MPU6050_Data_FIFO[0][2]=(float)MPU6050_Device_0_Data.Accel_X/MPU6050_ACCEL_SENS_4;
			MPU6050_Data_FIFO[1][2]=(float)MPU6050_Device_0_Data.Accel_Y/MPU6050_ACCEL_SENS_4;
			MPU6050_Data_FIFO[2][2]=(float)MPU6050_Device_0_Data.Accel_Z/MPU6050_ACCEL_SENS_4;
			//3 Axis Gyro Processing
			MPU6050_Data_FIFO[3][2]=(float)MPU6050_Device_0_Data.Gyro_X/MPU6050_GYRO_SENS_250;
			MPU6050_Data_FIFO[4][2]=(float)MPU6050_Device_0_Data.Gyro_Y/MPU6050_GYRO_SENS_250;
			MPU6050_Data_FIFO[5][2]=(float)MPU6050_Device_0_Data.Gyro_Z/MPU6050_GYRO_SENS_250;
		*/
		
		/*
			Velocity Data trapizoidal integration
			temp=(MPU6050_Data_FIFO[0][0]+4*MPU6050_Data_FIFO[0][1]+MPU6050_Data_FIFO[0][2]);
			Velocity_Data_FIFO[0][2]=Velocity_Data_FIFO[0][2]+0.0005f*temp;
			
			temp=(MPU6050_Data_FIFO[1][0]+4*MPU6050_Data_FIFO[1][1]+MPU6050_Data_FIFO[0][2]);
			Velocity_Data_FIFO[1][2]=Velocity_Data_FIFO[1][2]+0.0005f*temp;
			
			temp=(MPU6050_Data_FIFO[2][0]+4*MPU6050_Data_FIFO[2][1]+MPU6050_Data_FIFO[2][2]);
			Velocity_Data_FIFO[2][2]=Velocity_Data_FIFO[2][2]+0.0005f*temp;
		*/
		
		/*
			Rotation Data trapizoidal integration
			temp=(MPU6050_Data_FIFO[3][0]+4*MPU6050_Data_FIFO[3][1]+MPU6050_Data_FIFO[0][2]);
			Rotation_Data[0]=Rotation_Data[0]+0.0005f*temp;
			
			temp=(MPU6050_Data_FIFO[4][0]+4*MPU6050_Data_FIFO[4][1]+MPU6050_Data_FIFO[0][2]);
			Rotation_Data[1]=Rotation_Data[1]+0.0005f*temp;
			
			temp=(MPU6050_Data_FIFO[5][0]+4*MPU6050_Data_FIFO[5][1]+MPU6050_Data_FIFO[2][2]);
			Rotation_Data[2]=Rotation_Data[2]+0.0005f*temp;
		*/

		/*
			Displacement Data
			temp=(Velocity_Data_FIFO[0][0]+4*Velocity_Data_FIFO[0][1]+Velocity_Data_FIFO[0][2]);
			Displacement_Data[0]=Displacement_Data[0]+0.0005f*temp;
			
			temp=(Velocity_Data_FIFO[1][0]+4*Velocity_Data_FIFO[1][1]+Velocity_Data_FIFO[0][2]);
			Displacement_Data[1]=Displacement_Data[1]+0.0005f*temp;
			
			temp=(Velocity_Data_FIFO[2][0]+4*Velocity_Data_FIFO[2][1]+Velocity_Data_FIFO[2][2]);
			Displacement_Data[2]=Displacement_Data[2]+0.0005f*temp;
		*/
	}
	//Re-enable IRQ
	__enable_irq();
}

//This function handles program failures.
void Fail_Handler(void){
	printf("Critical Failure \n");
  while(1)
	{
  }
}
