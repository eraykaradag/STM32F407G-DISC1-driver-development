/*
 * stm32f4xx_usart.c
 *
 *  Created on: Apr 6, 2024
 *      Author: PC
 */

#include "stm32f4xx_usart.h"

static uint16_t AHB_PreScaler[9] = { 2, 4, 8, 16, 32, 64, 128, 256, 512 };
static uint8_t APB1_PreScaler[4] = { 2, 4, 8, 16 };
uint32_t RCC_GetPCLK1Value(void) {
	uint32_t pclk1, SystemClk;
	uint8_t clcksrc = ((RCC->CFGR >> 2) & 0x3);

	if (clcksrc == 0) {
		//HSI
		SystemClk = 16000000;
	} else if (clcksrc == 1) {
		//HSE
		SystemClk = 8000000;
	}
	uint8_t temp = ((RCC->CFGR >> 4) & 0xF), ahbpre, apb1pre;

	if (temp < 8) {
		ahbpre = 1;
	} else {
		ahbpre = AHB_PreScaler[temp - 8];
	}
	temp = ((RCC->CFGR >> 10) & 0x7);
	if (temp < 4) {
		apb1pre = 1;
	} else {
		apb1pre = APB1_PreScaler[temp - 4];
	}

	pclk1 = (SystemClk / ahbpre) / apb1pre;

	return pclk1;
}
uint32_t RCC_GetPCLK2Value(void) {
	uint32_t pclk2, SystemClk;
	uint8_t clcksrc = ((RCC->CFGR >> 2) & 0x3);

	if (clcksrc == 0) {
		//HSI
		SystemClk = 16000000;
	} else if (clcksrc == 1) {
		//HSE
		SystemClk = 8000000;
	}
	uint8_t temp = ((RCC->CFGR >> 4) & 0xF), ahbpre, apb2pre;

	if (temp < 0x08) {
		ahbpre = 1;
	} else {
		ahbpre = AHB_PreScaler[temp - 8];
	}
	temp = ((RCC->CFGR >> 13) & 0x7);
	if (temp < 0x04) {
		apb2pre = 1;
	} else {
		apb2pre = APB1_PreScaler[temp - 4];
	}

	pclk2 = (SystemClk / ahbpre) / apb2pre;

	return pclk2;
}

void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi){
	if (EnorDi == ENABLE) {
		if (pUSARTx == USART1) {
			USART1_PCLK_EN();
		} else if (pUSARTx == USART2) {
			USART2_PCLK_EN();
		} else if (pUSARTx == USART3) {
			USART3_PCLK_EN();
		} else if (pUSARTx == UART4) {
			UART4_PCLK_EN();
		} else if (pUSARTx == UART5) {
			UART5_PCLK_EN();
		} else if (pUSARTx == USART6) {
			USART6_PCLK_EN();
		}
	} else {
		if (pUSARTx == USART1) {
			USART1_PCLK_DI();
		} else if (pUSARTx == USART2) {
			USART2_PCLK_DI();
		} else if (pUSARTx == USART3) {
			USART3_PCLK_DI();
		} else if (pUSARTx == UART4) {
			UART4_PCLK_DI();
		} else if (pUSARTx == UART5) {
			UART5_PCLK_DI();
		} else if (pUSARTx == USART6) {
			USART6_PCLK_DI();
		}
	}
}


void USART_Init(USART_Handle_t *pUSARTHandle)
{


	uint32_t tempreg=0;


	 USART_PeriClockControl(pUSARTHandle->pUSARTx, ENABLE);

	if ( pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_RX)
	{
		tempreg|= (1 << USART_CR1_RE);
	}else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_TX)
	{
		tempreg |= ( 1 << USART_CR1_TE );

	}else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_TXRX)
	{
		tempreg |= ( ( 1 << USART_CR1_RE) | ( 1 << USART_CR1_TE) );
	}

	tempreg |= pUSARTHandle->USART_Config.USART_WordLength << USART_CR1_M ;


	if ( pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_EVEN)
	{
		tempreg |= ( 1 << USART_CR1_PCE);


	}else if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_ODD )
	{
	    tempreg |= ( 1 << USART_CR1_PCE);

	    tempreg |= ( 1 << USART_CR1_PS);

	}

	pUSARTHandle->pUSARTx->CR1 = tempreg;


	tempreg=0;

	tempreg |= pUSARTHandle->USART_Config.USART_NoOfStopBits << USART_CR2_STOP;

	pUSARTHandle->pUSARTx->CR2 = tempreg;


	tempreg=0;

	if ( pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS)
	{
		tempreg |= ( 1 << USART_CR3_CTSE);


	}else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_RTS)
	{
		tempreg |= ( 1 << USART_CR3_RTSE);

	}else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS_RTS)
	{
		tempreg |= (1 << USART_CR3_CTSE);
		tempreg |= (1 << USART_CR3_RTSE);
	}


	pUSARTHandle->pUSARTx->CR3 = tempreg;

/******************************** Configuration of BRR(Baudrate register)******************************************/

	//Implement the code to configure the baud rate
	//We will cover this in the lecture. No action required here
	USART_SetBaudRate(pUSARTHandle->pUSARTx,pUSARTHandle->USART_Config.USART_Baud);

}
void USART_DeInit(USART_RegDef_t *pUSARTx){
	if(pUSARTx == USART1){
		USART1_REG_RESET();
	}else if(pUSARTx == USART2){
		USART2_REG_RESET();
	}else if(pUSARTx == USART3){
		USART3_REG_RESET();
	}else if(pUSARTx == UART4){
		UART4_REG_RESET();
	}else if(pUSARTx == UART5){
		UART5_REG_RESET();
	}else if(pUSARTx == USART6){
		USART6_REG_RESET();
	}
}

void USART_SendData(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t Len){
	uint16_t *pdata;
	for (uint32_t i = 0; i < Len; i++) {
		while (!USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_TXE))
			;

		if (pUSARTHandle->USART_Config.USART_NoOfStopBits == USART_WORDLEN_9BITS) {
			pdata = (uint16_t*) pTxBuffer;
			pUSARTHandle->pUSARTx->DR = (*pdata & (uint16_t) 0x01FF);

			if (pUSARTHandle->USART_Config.USART_ParityControl
					== USART_PARITY_DISABLE) {
				pTxBuffer++;
				pTxBuffer++;
			} else {
				pTxBuffer++;
			}
		} else {
			pUSARTHandle->pUSARTx->DR = (*pTxBuffer & (uint8_t) 0xFF);

			pTxBuffer++;
		}
	}

	while (!USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_TC));
}
void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len){
	for (uint32_t i = 0; i < Len; i++) {
		while (!USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_RXNE))
					;

		if (pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS) {

			if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE) {

				*((uint16_t*) pRxBuffer) = (pUSARTHandle->pUSARTx->DR & (uint16_t) 0x1FF);

				pRxBuffer++;
				pRxBuffer++;
			} else {
				*pRxBuffer = (pUSARTHandle->pUSARTx->DR & (uint8_t) 0xFF);

				pRxBuffer++;
			}
		} else {

			if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE) {

				*pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);
			}

			else {

				*pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->DR & (uint8_t) 0x7F);
			}

			pRxBuffer++;
		}
	}
}
uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t Len){
	return 0;
}
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len){
	return 0;
}

void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi){}
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority){}
void USART_IRQHandling(USART_Handle_t *pHandle){}


void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi){
	if (EnorDi == ENABLE) {
		pUSARTx->CR1 |= (1<<USART_CR1_UE);
	} else {
		pUSARTx->CR1 |= ~(1<<USART_CR1_UE);
	}
}
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx , uint32_t FlagName){
	if (pUSARTx->SR & FlagName) {
		return 1;
	} else {
		return 0;
	}
}
void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate){
	//Variable to hold the APB clock
	uint32_t PCLKx;

	uint32_t usartdiv;

	//variables to hold Mantissa and Fraction values
	uint32_t M_part, F_part;

	uint32_t tempreg = 0;

	//Get the value of APB bus clock in to the variable PCLKx
	if (pUSARTx == USART1 || pUSARTx == USART6) {
		//USART1 and USART6 are hanging on APB2 bus
		PCLKx = RCC_GetPCLK2Value();
	} else {
		PCLKx = RCC_GetPCLK1Value();
	}

	//Check for OVER8 configuration bit
	if (pUSARTx->CR1 & (1 << USART_CR1_OVER8)) {
		//OVER8 = 1 , over sampling by 8
		usartdiv = ((25 * PCLKx) / (2 * BaudRate));
	} else {
		//over sampling by 16
		usartdiv = ((25 * PCLKx) / (4 * BaudRate));
	}

	//Calculate the Mantissa part
	M_part = usartdiv / 100;

	//Place the Mantissa part in appropriate bit position . refer USART_BRR
	tempreg |= M_part << 4;

	//Extract the fraction part
	F_part = (usartdiv - (M_part * 100));

	//Calculate the final fractional
	if (pUSARTx->CR1 & (1 << USART_CR1_OVER8)) {
		//OVER8 = 1 , over sampling by 8
		F_part = (((F_part * 8) + 50) / 100) & ((uint8_t) 0x07);

	} else {
		//over sampling by 16
		F_part = (((F_part * 16) + 50) / 100) & ((uint8_t) 0x0F);

	}

	//Place the fractional part in appropriate bit position . refer USART_BRR
	tempreg |= F_part;

	//copy the value of tempreg in to BRR register
	pUSARTx->BRR = tempreg;
}
void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint16_t StatusFlagName){
	pUSARTx->SR &= ~(StatusFlagName);
}


void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle,uint8_t AppEv){

}
