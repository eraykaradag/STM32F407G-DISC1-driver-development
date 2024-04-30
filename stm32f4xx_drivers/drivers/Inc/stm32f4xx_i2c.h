/*
 * stm32f4xx_i2c.h
 *
 *  Created on: Feb 4, 2024
 *      Author: PC
 */

#ifndef INC_STM32F4XX_I2C_H_
#define INC_STM32F4XX_I2C_H_


#include "stm32f4xx.h"

typedef struct{
	uint32_t I2C_SCLSpeed;
	uint8_t I2C_DeviceAddr;
	uint8_t I2C_AckCtrl;
	uint16_t I2C_FMDutyCycle;
}I2C_Config_t;

typedef struct{
	I2C_Config_t I2C_Config;
	I2C_RegDef_t *pI2Cx;
	uint8_t *pTxBuffer;
	uint8_t *pRxBuffer;
	uint32_t TxLen;
	uint32_t RxLen;
	uint8_t TxRxState;
	uint8_t DevAddr;
	uint32_t RxSize;
	uint8_t Sr;
}I2C_Handle_t;

//SR STATES
#define I2C_SR_DISABLE 0
#define I2C_SR_ENABLE 1
//I2C APP STATES
#define I2C_READY 0
#define I2C_BUSY_RX 1
#define I2C_BUSY_TX 2
//SCL speed

#define I2C_SCL_SPEED_SM 100000
#define I2C_SCL_SPEED_FM 400000

//ack control
#define I2C_ACK_EN 1
#define I2C_ACK_DI 0

//fm duty cycle
#define I2C_FM_DUTY_2		0
#define I2C_FM_DUTY_16_9	1

//I2C bit positions
#define I2C_SR1_TXE 7
#define I2C_SR1_RXNE 6
#define I2C_SR1_SB 0
#define I2C_SR1_ADDR 1
#define I2C_SR1_BTF 2
#define I2C_SR1_STOPF 4
#define I2C_SR1_AF 10
#define I2C_SR2_BUSY 1
//I2C flags
#define I2C_FLAG_TXE (1<<I2C_SR1_TXE)
#define I2C_FLAG_RXNE (1<<I2C_SR1_RXNE)
#define I2C_FLAG_SB (1<<I2C_SR1_SB)
#define I2C_FLAG_ADDR (1<<I2C_SR1_ADDR)
#define I2C_FLAG_BTF (1<<I2C_SR1_BTF)
#define I2C_FLAG_STOPF (1<<I2C_SR1_STOPF)

//app events
#define I2C_EV_TX_CMPLT 0
#define I2C_EV_RX_CMPLT 1
#define I2C_EV_STOP		2

//I2C errors
#define I2C_ERROR_BERR  3
#define I2C_ERROR_ARLO  4
#define I2C_ERROR_AF    5
#define I2C_ERROR_OVR   6
#define I2C_ERROR_TIMEOUT 7


void I2C_PClkControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);

void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);


//send
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle,uint8_t* pTxBuffer, uint32_t Len, uint8_t SlaveAddr);
//send with interrupt
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle,uint8_t* pTxBuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t Sr);
void I2C_CloseTransmission(I2C_Handle_t *pI2CHandle);
//recv
void I2C_MasterRecieveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer,uint32_t Len, uint8_t SlaveAddr);
//with interrupt
uint8_t I2C_MasterRecieveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer,uint32_t Len, uint8_t SlaveAddr,uint8_t Sr);
void I2C_CloseRecieving(I2C_Handle_t *pI2CHandle);
//irq config
void I2C_IRQITConfig(uint8_t IRQNum, uint8_t EnorDi);
void I2C_IRQPriorityConfig(uint8_t IRQNum, uint32_t IRQPriority);
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle);
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle);


void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx,uint8_t EnorDi);
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx,uint32_t flagName);
void I2C_ManageAck(I2C_RegDef_t *pI2Cx,uint8_t EnorDi);
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv);

#endif /* INC_STM32F4XX_I2C_H_ */
