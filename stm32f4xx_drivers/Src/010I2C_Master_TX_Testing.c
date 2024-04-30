/*
 * 010I2C_Master_TX_Testing.c
 *
 *  Created on: Feb 25, 2024
 *      Author: PC
 */
#include <stdio.h>
#include <string.h>
#include "stm32f4xx.h"

//arduino addr
#define SLAVEADDR 0x68
//test data

uint8_t data[] = "Testing I2C master tx\n";

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
	I2C_Pins.GPIO_PinConfig.GPIO_PinNum = GPIO_P9;
	GPIO_Init(&I2C_Pins);

}
void I2C1_Init(void) {
	I2C_Handle.pI2Cx = I2C1;
	I2C_Handle.I2C_Config.I2C_AckCtrl = I2C_ACK_EN;
	I2C_Handle.I2C_Config.I2C_DeviceAddr = 0x61;
	I2C_Handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
	I2C_Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;

	I2C_Init(&I2C_Handle);
}

void Button_Init(void) {
	GPIO_Handle_t Button_Pins;
	Button_Pins.pGPIOx = GPIOA;
	Button_Pins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	Button_Pins.GPIO_PinConfig.GPIO_PinPUPDControl = GPIO_NO_PUPD;
	Button_Pins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	Button_Pins.GPIO_PinConfig.GPIO_PinNum = GPIO_P0;

	GPIO_Init(&Button_Pins);
}
int main(void) {
	Button_Init();
	//GPIO PINS SET FOR THE I2C PERIPH
	I2C1_GPIOInit();

	//I2C PERIPHERAL CONFIGURATION
	I2C1_Init();

	//ENABLE I2C1
	I2C_PeripheralControl(I2C1, ENABLE);

	while(1){
		while(!GPIO_ReadFromInputPin(GPIOA, GPIO_P0));
		delay();
		I2C_MasterSendData(&I2C_Handle, data, strlen((char*) data), SLAVEADDR);
	}

}

