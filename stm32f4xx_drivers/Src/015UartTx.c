/*
 * 015UartTx.c
 *
 *  Created on: Apr 6, 2024
 *      Author: PC
 */

#include "stm32f4xx.h"
#include <stdio.h>
#include <string.h>

char msg[1024] = "UART Tx Test.\n\r";
USART_Handle_t UART_Handle;

void USARTx_GPIOInit(void) {
	GPIO_Handle_t USART_Pins;
	USART_Pins.pGPIOx = GPIOA;
	USART_Pins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	USART_Pins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	USART_Pins.GPIO_PinConfig.GPIO_PinPUPDControl = GPIO_PU;
	USART_Pins.GPIO_PinConfig.GPIO_PinAltFunMode = 7;
	USART_Pins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//TX INIT
	USART_Pins.GPIO_PinConfig.GPIO_PinNum = GPIO_P2;
	GPIO_Init(&USART_Pins);

	//RX INIT
	USART_Pins.GPIO_PinConfig.GPIO_PinNum = GPIO_P3;
	GPIO_Init(&USART_Pins);

}

void USARTx_Init(void) {
	UART_Handle.pUSARTx = USART2;
	UART_Handle.USART_Config.USART_Baud = USART_STD_BAUD_115200;
	UART_Handle.USART_Config.USART_ParityControl = USART_PARITY_DISABLE;
	UART_Handle.USART_Config.USART_WordLength = USART_WORDLEN_8BITS;
	UART_Handle.USART_Config.USART_NoOfStopBits = USART_STOPBITS_1;
	UART_Handle.USART_Config.USART_Mode = USART_MODE_TXRX;
	UART_Handle.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;
	USART_Init(&UART_Handle);
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
void delay() {
	for (int i = 0; i < 500000 / 2; i++)
		;
}
int main(void){
	Button_Init();
	USARTx_GPIOInit();
	USARTx_Init();
	USART_PeripheralControl(USART2, ENABLE);

	while(1){
		while(!GPIO_ReadFromInputPin(GPIOA, GPIO_P0));
		USART_SendData(&UART_Handle, (uint8_t*)msg, strlen(msg));
		delay();
	}

	return 0;
}
