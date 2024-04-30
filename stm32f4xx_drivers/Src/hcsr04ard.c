/*
 * 015UartTx.c
 *
 *  Created on: Apr 6, 2024
 *      Author: PC
 */

#include "stm32f4xx.h"
#include <stdio.h>
#include <string.h>

USART_Handle_t UART_Handle;
void delay(uint32_t time){
	uint32_t newtime = time*24;
	while(newtime--);
}
void TIM2_us_Delay(uint32_t delay){
	RCC->APB1ENR |= 1;
	TIM2->ARR = (int)(delay/0.0625);
	TIM2->CNT = 0;
	TIM2->CR1 |= 1;

	while(!(TIM2->SR & (1<<0)));

	TIM2->SR &= ~(0X0001);
}
/*void USARTx_GPIOInit(void) {
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
}*/
/*void Button_Init(void) {
	GPIO_Handle_t Button_Pins;
	Button_Pins.pGPIOx = GPIOA;
	Button_Pins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	Button_Pins.GPIO_PinConfig.GPIO_PinPUPDControl = GPIO_NO_PUPD;
	Button_Pins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	Button_Pins.GPIO_PinConfig.GPIO_PinNum = GPIO_P0;

	GPIO_Init(&Button_Pins);
}*/
void HCSR04_Init(void) {
	GPIO_Handle_t HCSR04Pins;
	HCSR04Pins.pGPIOx = GPIOB;
	HCSR04Pins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	HCSR04Pins.GPIO_PinConfig.GPIO_PinPUPDControl = GPIO_NO_PUPD;
	HCSR04Pins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	HCSR04Pins.GPIO_PinConfig.GPIO_PinNum = GPIO_P5;

	//echopin
	GPIO_Init(&HCSR04Pins);

	//trig pin
	HCSR04Pins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	HCSR04Pins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	HCSR04Pins.GPIO_PinConfig.GPIO_PinNum = GPIO_P7;

	GPIO_Init(&HCSR04Pins);

}
uint32_t HCSR04_Read(){

	uint32_t data=0;
	uint32_t dist =-1;
	GPIO_ResetBSRR(GPIOB, GPIO_P7);
	TIM2_us_Delay(2);
	GPIO_SetBSRR(GPIOB, GPIO_P7);
	TIM2_us_Delay(10);
	GPIO_ResetBSRR(GPIOB, GPIO_P7);

	while(GPIO_ReadFromInputPin(GPIOB, GPIO_P5)){
		data++;
		TIM2_us_Delay(1);
	}
	if(data > 0){
		data = data*(0.0625*0.000001);
		dist = ((data*340)/2)*100;
	}
	TIM2_us_Delay(4);
	return dist;

}
int main(void){
	HCSR04_Init();
	//USARTx_GPIOInit();
	//USARTx_Init();
	//USART_PeripheralControl(USART2, ENABLE);
	uint32_t msg = -1;
	while(1){
		msg = HCSR04_Read();
		printf("distance: %lu \n", msg);
		delay(5000);
	}

	return 0;
}
