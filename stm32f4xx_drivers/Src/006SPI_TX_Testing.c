/*
 * 006SPI_TX_Testing.c
 *
 *  Created on: Mar 17, 2024
 *      Author: PC
 */
#include "stm32f4xx.h"
#define MPU6500_WHO_AM_I    0x75
#define MPU6500_GYRO_XOUT_H 0x43
#define MPU6500_GYRO_XOUT_L 0x44
#define MPU6500_GYRO_YOUT_H 0x45
#define MPU6500_GYRO_YOUT_L 0x46
#define MPU6500_GYRO_ZOUT_H 0x47
#define MPU6500_GYRO_ZOUT_L 0x48
#define REG_USER_CTRL    	0x6A

void SPI2_GPIOInit(void) {
	GPIO_Handle_t SPI_Pins;
	SPI_Pins.pGPIOx = GPIOB;
	SPI_Pins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPI_Pins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPI_Pins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPI_Pins.GPIO_PinConfig.GPIO_PinPUPDControl = GPIO_NO_PUPD;
	SPI_Pins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//SCLK
	SPI_Pins.GPIO_PinConfig.GPIO_PinNum = GPIO_P13;
	GPIO_Init(&SPI_Pins);

	//MOSI
	SPI_Pins.GPIO_PinConfig.GPIO_PinNum = GPIO_P15;
	GPIO_Init(&SPI_Pins);

	//MISO
	SPI_Pins.GPIO_PinConfig.GPIO_PinNum = GPIO_P14;
	GPIO_Init(&SPI_Pins);

	//NSS
	SPI_Pins.GPIO_PinConfig.GPIO_PinNum = GPIO_P12;
	GPIO_Init(&SPI_Pins);
}
void SPI2_Init(void){
	SPI_Handle_t SPI2Handle;
	SPI2Handle.pSPIx = SPI2;

	SPI2Handle.SPIConfig.SPI_BusConfig = SPI_BUS_CFG_FD;
	SPI2Handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2Handle.SPIConfig.SPI_SClkSpeed = SPI_SCLK_SPEED_DIV32;
	SPI2Handle.SPIConfig.SPI_DFF = SPI_DFF_8BIT;
	SPI2Handle.SPIConfig.SPI_CPOL = SPI_CPOL_HI;
	SPI2Handle.SPIConfig.SPI_CPHA = SPI_CPHA_HI;
	SPI2Handle.SPIConfig.SPI_SSM = SPI_SSM_DI;

	SPI_Init(&SPI2Handle);
}
void MPU6500_WriteRegister(uint8_t regAddr, uint8_t cmd){
	SPI_SendData(SPI2, &regAddr, 1);
	SPI_SendData(SPI2, &cmd, 1);
}
void MPU_Init(void){
	MPU6500_WriteRegister(0x6B, 0x80);
	delay();
	MPU6500_WriteRegister(0x6B, 0x03);
	delay();
	MPU6500_WriteRegister(0x6A, 0x10);
	delay();
	MPU6500_WriteRegister(0x6C, 0x00);
	delay();
	MPU6500_WriteRegister(0x19, 0);
	delay();
}
void MPU6500_ReadGyro(int16_t *gyroX, int16_t *gyroY, int16_t *gyroZ) {
    uint8_t data[6];

    // Send read commands for gyroscope data
    MPU6500_ReadRegister(MPU6500_GYRO_XOUT_H, data, 6);

    // Extract gyro data (assuming 16-bit data format)
    *gyroX = (int16_t)((data[0] << 8) | data[1]);
    *gyroY = (int16_t)((data[2] << 8) | data[3]);
    *gyroZ = (int16_t)((data[4] << 8) | data[5]);
}

void MPU6500_ReadRegister(uint8_t regAddr, uint8_t *data, uint8_t size) {
    // Send command to read from register
    uint8_t buffer = (regAddr | 0x80);
    uint8_t dummy = 0x00; // Send dummy byte to receive data
    SPI_SendData(SPI2, &buffer, 1);
    delay();
    SPI_SendData(SPI2, &dummy, 1);
    SPI_RecieveData(SPI2,data, 1);
    // Read data from register
    /*for (int i = 0; i < size; i++) {
        SPI_SendData(SPI2, &dummy, 1);
        SPI_RecieveData(SPI2, &data[i], 1);
    }*/
}
void delay(void) {
	for (int i = 0; i < 500000 / 2; i++)
		;
}

int main(void){
	int16_t gyroX, gyroY, gyroZ;
	uint8_t temp;
	SPI2_GPIOInit();
	SPI2_Init();
	SPI_SSOEConfig(SPI2, ENABLE);
	SPI_PeripheralControl(SPI2, ENABLE);
	MPU_Init();
	MPU6500_ReadRegister(0x75,&temp,1);
	/*while (1) {
	    MPU6500_ReadGyro(&gyroX, &gyroY, &gyroZ);
	    delay();
	}*/
    SPI_PeripheralControl(SPI2, DISABLE);
	return 0;
}
