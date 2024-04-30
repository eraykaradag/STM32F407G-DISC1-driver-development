/*
 * 010I2C_Master_TX_Testing.c
 *
 *  Created on: Feb 25, 2024
 *      Author: PC
 */
#include "stm32f4xx.h"
#include "stdio.h"
#include "stdbool.h"
#define SLAVEADDRGYRO 0x68
#define MPU6500_WHO_AM_I    	0x75
#define MPU6500_PWR_MGMT_1      0x6B
#define MPU6500_ACCEL_XOUT_H    0x3B
#define MPU6500_GYRO_XOUT_H     0x43
#define GYRO_SENSITIVITY    	131.0f

void delay() {
	for (int i = 0; i < 500000 / 2; i++)
		;
}

I2C_Handle_t I2C_Handle;

void I2C1_GPIOInit(void) {
	GPIO_Handle_t I2C_Pins;
	I2C_Pins.pGPIOx = GPIOB;
	I2C_Pins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	I2C_Pins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	I2C_Pins.GPIO_PinConfig.GPIO_PinPUPDControl = GPIO_PU;
	I2C_Pins.GPIO_PinConfig.GPIO_PinAltFunMode = 4;
	I2C_Pins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//SCL INIT
	I2C_Pins.GPIO_PinConfig.GPIO_PinNum = GPIO_P6;
	GPIO_Init(&I2C_Pins);

	//SDA INIT
	I2C_Pins.GPIO_PinConfig.GPIO_PinNum = GPIO_P7;
	GPIO_Init(&I2C_Pins);

}
void I2C1_Init(void) {
	I2C_Handle.pI2Cx = I2C1;
	I2C_Handle.I2C_Config.I2C_AckCtrl = I2C_ACK_EN;
	I2C_Handle.I2C_Config.I2C_DeviceAddr = 0x44;
	I2C_Handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
	I2C_Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;

	I2C_Init(&I2C_Handle);
}
void MPU6500_ReadGyro(int16_t* gyrox,int16_t* gyroy,int16_t* gyroz){
	uint8_t cmdcode = MPU6500_GYRO_XOUT_H;
	uint8_t hi,lo;

	I2C_MasterSendData(&I2C_Handle, &cmdcode, 1, SLAVEADDRGYRO);

	I2C_MasterRecieveData(&I2C_Handle, &hi, 1, SLAVEADDRGYRO);
	I2C_MasterRecieveData(&I2C_Handle, &lo, 1, SLAVEADDRGYRO);
	*gyrox = (int16_t) ((hi << 8) | lo);

	I2C_MasterRecieveData(&I2C_Handle, &hi, 1, SLAVEADDRGYRO);
	I2C_MasterRecieveData(&I2C_Handle, &lo, 1, SLAVEADDRGYRO);
	*gyroy = (int16_t) ((hi << 8) | lo);

	I2C_MasterRecieveData(&I2C_Handle, &hi, 1, SLAVEADDRGYRO);
	I2C_MasterRecieveData(&I2C_Handle, &lo, 1, SLAVEADDRGYRO);
	*gyroz = (int16_t) ((hi << 8) | lo);
}

int main(void) {
	int16_t rawgyrox,rawgyroy,rawgyroz;

	//GPIO PINS SET FOR THE I2C PERIPH
	I2C1_GPIOInit();

	//I2C PERIPHERAL CONFIGURATION
	I2C1_Init();

	//ENABLE I2C1
	I2C_PeripheralControl(I2C1, ENABLE);

	I2C_ManageAck(I2C1, ENABLE);


	/*while(1){
		MPU6500_ReadGyro(&rawgyrox, &rawgyroy, &rawgyroz);
		printf("gyrox = %d, gyroy = %d, gyroz = %d \n", rawgyrox, rawgyroy,rawgyroz);
		delay();
	}*/

	return 0;

}

