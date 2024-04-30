/*
 * stm32f4xx_gpio.h
 *
 *  Created on: Jan 27, 2024
 *      Author: PC
 */

#ifndef INC_STM32F4XX_GPIO_H_
#define INC_STM32F4XX_GPIO_H_

#include "stm32f4xx.h"

typedef struct{
	uint8_t GPIO_PinNum;
	uint8_t GPIO_PinMode;
	uint8_t GPIO_PinSpeed;
	uint8_t GPIO_PinPUPDControl;
	uint8_t GPIO_PinOPType;
	uint8_t GPIO_PinAltFunMode;
}GPIO_PinConfig_t;

typedef struct{
	GPIO_RegDef_t *pGPIOx;
	GPIO_PinConfig_t GPIO_PinConfig;
}GPIO_Handle_t;

#define GPIO_P0 0
#define GPIO_P1 1
#define GPIO_P2 2
#define GPIO_P3 3
#define GPIO_P4 4
#define GPIO_P5 5
#define GPIO_P6 6
#define GPIO_P7 7
#define GPIO_P8 8
#define GPIO_P9 9
#define GPIO_P10 10
#define GPIO_P11 11
#define GPIO_P12 12
#define GPIO_P13 13
#define GPIO_P14 14
#define GPIO_P15 15


#define GPIO_MODE_IN 0
#define GPIO_MODE_OUT 1
#define GPIO_MODE_ALTFN 2
#define GPIO_MODE_ANALOG 3
#define GPIO_MODE_IT_FT 4
#define GPIO_MODE_IT_RT 5
#define GPIO_MODE_IT_RFT 6

#define GPIO_OP_TYPE_PP 0
#define GPIO_OP_TYPE_OD 1

#define GPIO_SPEED_LOW 0
#define GPIO_SPEED_MED 1
#define GPIO_SPEED_FAST 2
#define GPIO_SPEED_HIGH 3

#define GPIO_NO_PUPD 0
#define GPIO_PU 1
#define GPIO_PD 2

void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);
void GPIO_PClkControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteFromOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t val);
void GPIO_WriteFromOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t val);
void GPIO_SetBSRR(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
void GPIO_ResetBSRR(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

void GPIO_IRQITConfig(uint8_t IRQNum, uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNum, uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);
#endif /* INC_STM32F4XX_GPIO_H_ */
