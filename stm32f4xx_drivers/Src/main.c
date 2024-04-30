#include "stm32f4xx.h"
#include <stdio.h>
#include <string.h>
void delay(void){
	for(int i = 0; i<500000/2;i++);
}
int main(void){
	GPIO_Handle_t GpioLED;
	GPIO_Handle_t GpioButton;
	memset(&GpioLED,0,sizeof(GpioLED));
	memset(&GpioButton,0,sizeof(GpioButton));

	GpioLED.pGPIOx = GPIOD;
	GpioLED.GPIO_PinConfig.GPIO_PinNum = GPIO_P12;
	GpioLED.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLED.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLED.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLED.GPIO_PinConfig.GPIO_PinPUPDControl = GPIO_NO_PUPD;

	GpioButton.pGPIOx = GPIOA;
	GpioButton.GPIO_PinConfig.GPIO_PinNum = GPIO_P0;
	GpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_RT;
	GpioButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioButton.GPIO_PinConfig.GPIO_PinPUPDControl = GPIO_NO_PUPD;
	GPIO_PClkControl(GPIOA, ENABLE);
	GPIO_PClkControl(GPIOD, ENABLE);
	GPIO_Init(&GpioButton);
	GPIO_Init(&GpioLED);

	GPIO_IRQITConfig(IRQ_NO_EXTI0, ENABLE);
	return 0;
}
void EXTI0_IRQHandler(void){
	GPIO_IRQHandling(0);
	GPIO_ToggleOutputPin(GPIOD, GPIO_P12);
}
