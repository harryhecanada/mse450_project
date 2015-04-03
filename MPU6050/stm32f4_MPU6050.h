#ifndef __MPU6050_H
#define __MPU6050_H

//Includes
#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_i2c.h"

//MPU6050 Maximum Clock Speed
#define MPU6050_I2C_CLOCK			400000

//MPU6050 Default Address
#define MPU6050_I2C_ADDR			0xD0

//MPU6050 WHO AM I Register
#define MPU6050_I_AM				0x68

//Define meaning of ACK and NACK
#define ACK	0x01
#define NACK 0x00

//MPU6050 Registers
#define MPU6050_AUX_VDDIO			0x01
#define MPU6050_SMPLRT_DIV			0x19
#define MPU6050_CONFIG				0x1A
#define MPU6050_GYRO_CONFIG			0x1B
#define MPU6050_ACCEL_CONFIG		0x1C
#define MPU6050_MOTION_THRESH		0x1F
#define MPU6050_FIFO_EN					0x23
#define MPU6050_INT_PIN_CFG			0x37
#define MPU6050_INT_ENABLE			0x38
#define MPU6050_INT_STATUS			0x3A
#define MPU6050_ACCEL_XOUT_H		0x3B
#define MPU6050_ACCEL_XOUT_L		0x3C
#define MPU6050_ACCEL_YOUT_H		0x3D
#define MPU6050_ACCEL_YOUT_L		0x3E
#define MPU6050_ACCEL_ZOUT_H		0x3F
#define MPU6050_ACCEL_ZOUT_L		0x40
#define MPU6050_TEMP_OUT_H			0x41
#define MPU6050_TEMP_OUT_L			0x42
#define MPU6050_GYRO_XOUT_H			0x43
#define MPU6050_GYRO_XOUT_L			0x44
#define MPU6050_GYRO_YOUT_H			0x45
#define MPU6050_GYRO_YOUT_L			0x46
#define MPU6050_GYRO_ZOUT_H			0x47
#define MPU6050_GYRO_ZOUT_L			0x48
#define MPU6050_MOT_DETECT_STATUS	0x61
#define MPU6050_SIGNAL_PATH_RESET	0x68
#define MPU6050_MOT_DETECT_CTRL		0x69
#define MPU6050_USER_CTRL			0x6A
#define MPU6050_PWR_MGMT_1			0x6B
#define MPU6050_PWR_MGMT_2			0x6C
#define MPU6050_FIFO_COUNTH			0x72
#define MPU6050_FIFO_COUNTL			0x73
#define MPU6050_FIFO_R_W			0x74
#define MPU6050_WHO_AM_I			0x75

//Gyro multipliers degrees/s
#define MPU6050_GYRO_SENS_250		((float) 32767/250)
#define MPU6050_GYRO_SENS_500		((float) 32767/500)
#define MPU6050_GYRO_SENS_1000		((float) 32767/1000)
#define MPU6050_GYRO_SENS_2000		((float) 32767/2000)

//Acce multipliers m/s/s
#define MPU6050_ACCEL_SENS_2			((float) 32767/2)
#define MPU6050_ACCEL_SENS_4			((float) 32767/4)
#define MPU6050_ACCEL_SENS_8			((float) 32767/8)
#define MPU6050_ACCEL_SENS_16		((float) 32767/16)

//Digital low pass filter configurations
#define MPU6050_DLFP_256Hz 0x00
#define MPU6050_DLFP_184Hz 0x01
#define MPU6050_DLFP_94Hz 0x02
#define MPU6050_DLFP_44Hz 0x03
#define MPU6050_DLFP_21Hz 0x04
#define MPU6050_DLFP_10Hz 0x05
#define MPU6050_DLFP_5Hz 0x06

//FIFO buffer configurations
#define MPU6050_FIFO_BUFFER_EN 0x40
#define MPU6050_FIFO_TEMP_EN 0x80
#define MPU6050_FIFO_XGYRO_EN 0x40
#define MPU6050_FIFO_YGYRO_EN 0x20
#define MPU6050_FIFO_ZGYRO_EN 0x10
#define MPU6050_FIFO_ACCEL_EN 0x08

//Interrupt Configurations
#define MPU6050_DATA_RDY_INT 0x01
#define MPU6050_FIFO_OFLOW_INT 0x10


//Address Types
typedef enum {
	MPU6050_Device_0 = 0xD0,
	MPU6050_Device_1 = 0xD2
} MPU6050_Addr;
//Error Types
typedef enum {
	MPU6050_OK = 0x00,
	MPU6050_NOT_CONNECTED = 0x01,
	MPU6050_WRONG_DEVICE = 0x02,
	MPU6050_UNKNOWN_ERROR = 0x03
} MPU6050_ERID;
//Accel Config Types
typedef enum {
	MPU6050_Accel_2G = 0x00,
	MPU6050_Accel_4G = 0x08,
	MPU6050_Accel_8G = 0x10,
	MPU6050_Accel_16G = 0x18
} MPU6050_Accel_Config;
//Gyro Config Types
typedef enum {
	MPU6050_Gyro_250ds = 0x00,
	MPU6050_Gyro_500ds = 0x08,
	MPU6050_Gyro_1000ds = 0x10,
	MPU6050_Gyro_2000ds = 0x18
} MPU6050_Gyro_Config;
//MPU6050 Data structure
typedef struct {
	int16_t Accel_X;
	int16_t Accel_Y;
	int16_t Accel_Z;
	int16_t Gyro_X;
	int16_t Gyro_Y;
	int16_t Gyro_Z;
} MPU6050_Data;

//Public Functions
MPU6050_ERID MPU6050_Init(I2C_TypeDef* I2Cx, MPU6050_Addr DeviceAddr, MPU6050_Accel_Config AccelSensitivity, MPU6050_Gyro_Config GyroSensitivity);

MPU6050_ERID MPU6050_ReadAccel(I2C_TypeDef* I2Cx, MPU6050_Data* DataStruct, MPU6050_Addr Addr);

MPU6050_ERID MPU6050_ReadGyro(I2C_TypeDef* I2Cx, MPU6050_Data* DataStruct, MPU6050_Addr Addr);

MPU6050_ERID MPU6050_ReadAll(I2C_TypeDef* I2Cx, MPU6050_Data* DataStruct, MPU6050_Addr Addr);

MPU6050_ERID MPU6050_FIFO_Read(I2C_TypeDef* I2Cx, MPU6050_Data* DataStruct, MPU6050_Addr Addr);

uint8_t MPU6050_Start(I2C_TypeDef* I2Cx, uint8_t addr, uint8_t dir, uint8_t ack);

uint8_t MPU6050_Stop(I2C_TypeDef* I2Cx);

uint8_t MPU6050_Read(I2C_TypeDef* I2Cx, uint8_t addr, uint8_t reg_id, uint8_t ack);

uint8_t MPU6050_Write(I2C_TypeDef* I2Cx, uint8_t addr, uint8_t reg_id, uint8_t data);

uint8_t MPU6050_MultiRead(I2C_TypeDef* I2Cx, uint8_t addr, uint8_t reg_id, uint8_t* data, uint16_t count);

#endif
