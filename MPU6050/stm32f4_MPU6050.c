/*
MPU6050 for the STM32F4 Discovery Board (STM32F4VG)
HOW TO USE:
1)Create data structure, select I2C channel, etc.
2)Call the MPU6050_Init function and pass the data.
3)Use ReadAccel or Read Gyro Or ReadAll functions to obtain data.
*/

//Includes
#include "stm32f4_MPU6050.h"
#include <stdio.h>

GPIO_InitTypeDef GPIO_I2C_InitStruct;

//Timeout value
const uint16_t I2C_Timeout=20000;

MPU6050_ERID MPU6050_Init(I2C_TypeDef *I2Cx, MPU6050_Addr Addr, MPU6050_Accel_Config AccelSensitivity, MPU6050_Gyro_Config GyroSensitivity) {
	uint8_t temp;
	I2C_InitTypeDef I2C_InitStruct;

	//Initialize I2C 
	I2C_Cmd(I2Cx, DISABLE);
	I2C_Cmd(I2Cx, ENABLE);
	GPIO_I2C_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_I2C_InitStruct.GPIO_OType = GPIO_OType_OD;
	GPIO_I2C_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_I2C_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	
	//Pin Setup 
	if(I2Cx==I2C1)
	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
		
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
		//Use PB8 and PB9 for SCL and SDA
		GPIO_I2C_InitStruct.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;

		GPIO_Init(GPIOB, &GPIO_I2C_InitStruct);
		/*
		GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_I2C1);//SCL
		GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_I2C1);//SDA
		*/
		
	}
	else if(I2Cx==I2C2)
	{
		//RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
		//Cannot use I2C2, pins are blocked.
		return MPU6050_UNKNOWN_ERROR;
		
	}
	else if(I2Cx==I2C3)
	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C3, ENABLE);
		
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
		
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_I2C3);
		GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_I2C3);
		
		//SCL on PA8
		GPIO_I2C_InitStruct.GPIO_Pin = GPIO_Pin_8;
		GPIO_Init(GPIOA, &GPIO_I2C_InitStruct);
		
		//SDA on PC9
		GPIO_I2C_InitStruct.GPIO_Pin = GPIO_Pin_9;
		GPIO_Init(GPIOC, &GPIO_I2C_InitStruct);
	}

	//I2C Setup 
	I2C_InitStruct.I2C_ClockSpeed = 400000;//FAST Mode is 400000, Normal is 100000
	I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStruct.I2C_OwnAddress1 = 0x00;
	I2C_InitStruct.I2C_Ack = I2C_Ack_Disable;
	I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_Init(I2Cx, &I2C_InitStruct);
	
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_I2C1);//SCL
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_I2C1);//SDA
	
	//Check if device is connected 
	if (MPU6050_Start(I2Cx, Addr, I2C_Direction_Transmitter, ACK) ) {
		return MPU6050_NOT_CONNECTED;
	}
	MPU6050_Stop(I2Cx);
	printf("Device is Connected, Reading Address... \n");
	//Check WHO AM I register
	if (MPU6050_Read(I2Cx, Addr, MPU6050_WHO_AM_I,NACK) != MPU6050_I_AM) {
		//Return error 
		return MPU6050_WRONG_DEVICE;
	}
	printf("Address is correct, Configuring Device... \n");
	//Configurations:
	//Setup power management on MPU6050, set SLEEP and CYCLE to OFF, set CLK to reference X Gyro, turn OFF temperature sensor. (0x01
	
	if(MPU6050_Write(I2Cx, Addr, MPU6050_PWR_MGMT_1, 0x01)){
		return MPU6050_UNKNOWN_ERROR;
	}
	
	temp = MPU6050_Read(I2Cx, Addr, MPU6050_ACCEL_CONFIG, NACK);
	temp = (temp & 0xE7) | (uint8_t)AccelSensitivity << 3;
	MPU6050_Write(I2Cx, Addr, MPU6050_ACCEL_CONFIG, temp);
	
	temp = MPU6050_Read(I2Cx, Addr, MPU6050_GYRO_CONFIG, NACK);
	temp = (temp & 0xE7) | (uint8_t)GyroSensitivity << 3;
	MPU6050_Write(I2Cx, Addr, MPU6050_GYRO_CONFIG, temp);
	
	if(MPU6050_Write(I2Cx,	Addr,	MPU6050_CONFIG, MPU6050_DLFP_10Hz)){
		return MPU6050_UNKNOWN_ERROR;
	}
	//Enable FIFO Buffer
	if(MPU6050_Write(I2Cx, Addr, MPU6050_USER_CTRL, MPU6050_FIFO_BUFFER_EN)){
		return MPU6050_UNKNOWN_ERROR;
	}
	//Enable the writing of XYZ Gyro data and XYZ Accel data into the FIFO buffer
	//if(MPU6050_Write(I2Cx,	Addr,	MPU6050_FIFO_EN, (MPU6050_FIFO_XGYRO_EN|MPU6050_FIFO_YGYRO_EN|MPU6050_FIFO_ZGYRO_EN|MPU6050_FIFO_ACCEL_EN))){
	//	return MPU6050_UNKNOWN_ERROR;
	//}
	//Enable FIFO Interrupt, and Data Ready Interrupt
	//if(MPU6050_Write(I2Cx,	Addr,	MPU6050_INT_ENABLE, (MPU6050_DATA_RDY_INT|MPU6050_FIFO_OFLOW_INT))){
	//	return MPU6050_UNKNOWN_ERROR;
	//}
	//Set the low pass filter to 10Hz

	//Config accelerometer, for good measuresments on human hand use +-4G
	//if(MPU6050_Write(I2Cx, Addr, MPU6050_ACCEL_CONFIG, AccelSensitivity)){
	//	return MPU6050_UNKNOWN_ERROR;
	//}
	//Config gyroscope, for good measuresments on human hand use +-250Deg/s
	//if(MPU6050_Write(I2Cx, Addr, MPU6050_GYRO_CONFIG, GyroSensitivity)){
	//	return MPU6050_UNKNOWN_ERROR;
	//}
	//Return OK 
	return MPU6050_OK;
}

MPU6050_ERID MPU6050_ReadAccel(I2C_TypeDef *I2Cx, MPU6050_Data *DataStruct, MPU6050_Addr Addr) {
	uint8_t data[6];
	
	//Read accelerometer data 
	MPU6050_MultiRead(I2Cx, Addr, MPU6050_ACCEL_XOUT_H, data, 6);
	
	//Format 
	DataStruct->Accel_X = (int16_t)(data[0] << 8 | data[1]);	
	DataStruct->Accel_Y = (int16_t)(data[2] << 8 | data[3]);
	DataStruct->Accel_Z = (int16_t)(data[4] << 8 | data[5]);
	
	//Return OK 
	return MPU6050_OK;
}

MPU6050_ERID MPU6050_ReadGyro(I2C_TypeDef *I2Cx, MPU6050_Data *DataStruct, MPU6050_Addr Addr) {
	uint8_t data[6];
	
	//Read gyroscope data 
	MPU6050_MultiRead(I2Cx, Addr, MPU6050_GYRO_XOUT_H, data, 6);
	
	//Format 
	DataStruct->Gyro_X = (int16_t)(data[0] << 8 | data[1]);
	DataStruct->Gyro_Y = (int16_t)(data[2] << 8 | data[3]);
	DataStruct->Gyro_Z = (int16_t)(data[4] << 8 | data[5]);

	//Return OK 
	return MPU6050_OK;
}

MPU6050_ERID MPU6050_ReadAll(I2C_TypeDef *I2Cx, MPU6050_Data *DataStruct, MPU6050_Addr Addr) {
	uint8_t data[14];
	
	//Read 14 bytes, the temperature is disabled, so we ignore it.
	MPU6050_MultiRead(I2Cx, Addr, MPU6050_ACCEL_XOUT_H, data, 14);
	
	DataStruct->Accel_X = (int16_t)(data[0] << 8 | data[1]);	
	DataStruct->Accel_Y = (int16_t)(data[2] << 8 | data[3]);
	DataStruct->Accel_Z = (int16_t)(data[4] << 8 | data[5]);
	
	DataStruct->Gyro_X = (int16_t)(data[8] << 8 | data[9]);
	DataStruct->Gyro_Y = (int16_t)(data[10] << 8 | data[11]);
	DataStruct->Gyro_Z = (int16_t)(data[12] << 8 | data[13]);

	//Return OK 
	return MPU6050_OK;
}
MPU6050_ERID MPU6050_FIFO_Read(I2C_TypeDef *I2Cx, MPU6050_Data *DataStruct, MPU6050_Addr Addr) {
	uint8_t data[12];
	
	//Read 12 bytes since temperature is disabled
	MPU6050_MultiRead(I2Cx, Addr, MPU6050_FIFO_R_W, data, 14);
	
	DataStruct->Accel_X = (int16_t)(data[0] << 8 | data[1]);	
	DataStruct->Accel_Y = (int16_t)(data[2] << 8 | data[3]);
	DataStruct->Accel_Z = (int16_t)(data[4] << 8 | data[5]);
	
	DataStruct->Gyro_X = (int16_t)(data[6] << 8 | data[7]);
	DataStruct->Gyro_Y = (int16_t)(data[8] << 8 | data[9]);
	DataStruct->Gyro_Z = (int16_t)(data[10] << 8 | data[11]);

	//Return OK 
	return MPU6050_OK;
}
uint8_t MPU6050_Start(I2C_TypeDef *I2Cx, uint8_t addr, uint8_t dir, uint8_t ack) {
	int16_t temp=I2C_Timeout;
	while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY));
	//Attempt Initial Communication 
	I2C_GenerateSTART(I2Cx, ENABLE);
	while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT) && temp) {
		temp--;
		if (temp == 0x00) {
			return 1;
		}
	}
	
	//Enable ACK, disable when in continuous reading mode if nessesary
	if (ack) {
		I2C_AcknowledgeConfig(I2C1, ENABLE);
	}
	
	I2C_Send7bitAddress(I2Cx, addr, dir);
	//Check if message is transmitted correctly
	if (dir == I2C_Direction_Transmitter) {
		temp=I2C_Timeout;
		while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) && temp)  {
			temp--;
			if (temp == 0x00) {
				return 1;
			}
		}
	} else if (dir == I2C_Direction_Receiver) {
		temp=I2C_Timeout;
		while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED) && temp) {
			temp--;
			if (temp == 0x00) {
				return 1;
			}
		}
	}
	//Return OK 
	return 0;
}

uint8_t MPU6050_Stop(I2C_TypeDef *I2Cx) {	
	
	int16_t temp = I2C_Timeout;
	
	I2C_GenerateSTOP(I2Cx, ENABLE);
	//Finish transmitting last bit of data before stopping
	while (((!I2C_GetFlagStatus(I2Cx, I2C_FLAG_TXE)) || (!I2C_GetFlagStatus(I2Cx, I2C_FLAG_BTF))) && temp) {
		temp--;
		if (temp == 0x00) {
			return 1;
		}
	}
	/* Send STOP condition */
	I2C_GenerateSTOP(I2Cx, ENABLE);
	
	return 0;
}

uint8_t MPU6050_Read(I2C_TypeDef *I2Cx, uint8_t addr, uint8_t reg_id, uint8_t ack) {
	uint8_t received_data;
	int16_t temp = I2C_Timeout;
	
	//Send data to device
	MPU6050_Start(I2Cx, addr, I2C_Direction_Transmitter, NACK);
	while (!I2C_GetFlagStatus(I2Cx, I2C_FLAG_TXE) && temp) {
		temp--;
		if (temp == 0x00) {
			return 1;
		}
	}
	I2C_SendData(I2Cx, reg_id);
	MPU6050_Stop(I2Cx);
	
	//Prepare to read, send ack for continuous read, no ack for one time.
	MPU6050_Start(I2Cx, addr, I2C_Direction_Receiver, NACK);
	if(ack)
	{
		I2C_AcknowledgeConfig(I2Cx, ENABLE);
	}
	else
	{
		I2C_AcknowledgeConfig(I2Cx, DISABLE);
		I2C_GenerateSTOP(I2Cx, ENABLE);
	}
	
	//wait until data recieved.
	temp=I2C_Timeout;
	while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED) && temp) {
		temp--;
		if (temp == 0x00) {
			return 1;
		}
	}
	
	received_data = I2C_ReceiveData(I2Cx);
	
	return received_data;
}

uint8_t MPU6050_Write(I2C_TypeDef *I2Cx, uint8_t addr, uint8_t reg_id, uint8_t data) {
	int16_t temp = I2C_Timeout;
	MPU6050_Start(I2Cx, addr, I2C_Direction_Transmitter, 0);
	//Send the register ID for writing
	while (!I2C_GetFlagStatus(I2Cx, I2C_FLAG_TXE) && temp) {
		temp--;
		if (temp == 0x00) {
			return 1;
		}
	}
	temp=I2C_Timeout;
	I2C_SendData(I2Cx, reg_id);
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	//Write to the register
	while (!I2C_GetFlagStatus(I2Cx, I2C_FLAG_TXE) && temp) {
		temp--;
		if (temp == 0x00) {
			return 1;
		}
	}
	I2C_SendData(I2Cx, data);
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	MPU6050_Stop(I2Cx);
	return 0;
}

uint8_t MPU6050_MultiRead(I2C_TypeDef *I2Cx, uint8_t addr, uint8_t reg_id, uint8_t *data, uint16_t count) {
	int16_t temp = I2C_Timeout;
	uint8_t i;
	MPU6050_Start(I2Cx, addr, I2C_Direction_Transmitter, ACK);
	//Send the register ID for reading
	while (!I2C_GetFlagStatus(I2Cx, I2C_FLAG_TXE) && temp) {
		temp--;
		if (temp == 0x00) {
			return 1;
		}
	}
	temp=I2C_Timeout;
	I2C_SendData(I2Cx, reg_id);
	MPU6050_Stop(I2Cx);
	MPU6050_Start(I2Cx, addr, I2C_Direction_Receiver, ACK);
	
	//ACK for initial bits, stop the ACK later on for the last bit. Reading the FIFO_R_W will not increment the register counter but reading the other registers will.
	I2C_AcknowledgeConfig(I2Cx, ENABLE);
	for (i = 0; i < count; i++) {
		if (i == (count - 1)) {
			temp=I2C_Timeout;
			I2C_AcknowledgeConfig(I2Cx, DISABLE);
			I2C_GenerateSTOP(I2Cx, ENABLE);
			while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED) && temp) {
				temp--;
				if (temp == 0x00) {
				return 1;
				}	
			}
			data[i] = I2C_ReceiveData(I2Cx);
		} 
		else {
			temp=I2C_Timeout;
			while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED) && temp) {
					temp--;
					if (temp == 0x00) {
					return 1;
					}	
			}
			data[i] = I2C_ReceiveData(I2Cx);
		}
	}
	return 0;
}



