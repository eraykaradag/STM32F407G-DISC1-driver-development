/*
 * stm32f4xx_gpio.c
 *
 *  Created on: Jan 27, 2024
 *      Author: PC
 */
#include "stm32f4xx_gpio.h"
#include <stdio.h>


void GPIO_Init(GPIO_Handle_t *pGPIOHandle){
	uint32_t temp = 0;

	//CLOCK ENABLING

	GPIO_PClkControl(pGPIOHandle->pGPIOx, ENABLE);
	//mod
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG){
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2* pGPIOHandle->GPIO_PinConfig.GPIO_PinNum));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << 2*(pGPIOHandle->GPIO_PinConfig.GPIO_PinNum));
		pGPIOHandle->pGPIOx->MODER |= temp;
		temp = 0;
	}
	else{
		//for interrupts
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT){
			EXTI->FTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNum);
			EXTI->RTSR &= ~(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNum);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT){
			EXTI->RTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNum);
			EXTI->FTSR &= ~(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNum);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT){
			EXTI->RTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNum);
			EXTI->FTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNum);
		}
		//SYSCFG
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNum / 4;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNum % 4;
		uint8_t portcode = GPIO_BASE_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] |= portcode << (4*temp2);
		//mask register
		EXTI->IMR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNum);
	}
	temp = 0;
	//speed
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2* pGPIOHandle->GPIO_PinConfig.GPIO_PinNum));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << 2*(pGPIOHandle->GPIO_PinConfig.GPIO_PinNum));
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;
	temp = 0;
	//pupdr
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPUPDControl << (2* pGPIOHandle->GPIO_PinConfig.GPIO_PinNum));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << 2*(pGPIOHandle->GPIO_PinConfig.GPIO_PinNum));
	pGPIOHandle->pGPIOx->PUPDR |= temp;
	temp = 0;
	//output type
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNum));
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNum));
	pGPIOHandle->pGPIOx->OTYPER |= temp;
	temp = 0;
	//Alternate function mode
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN){
		uint32_t temp1,temp2;
		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNum/8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNum%8;

		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4*temp2));

		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4*temp2));
		pGPIOHandle->pGPIOx->AFR[temp1] |= temp;

		temp = 0;
	}

}
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx){
	if(pGPIOx == GPIOA){
		GPIOA_REG_RESET();
	}
	else if(pGPIOx == GPIOB){
		GPIOB_REG_RESET();
	}
	else if(pGPIOx == GPIOC){
		GPIOC_REG_RESET();
	}
	else if(pGPIOx == GPIOD){
		GPIOD_REG_RESET();
	}
	else if(pGPIOx == GPIOE){
		GPIOE_REG_RESET();
	}
	else if(pGPIOx == GPIOF){
		GPIOF_REG_RESET();
	}
	else if(pGPIOx == GPIOG){
		GPIOG_REG_RESET();
	}
	else if(pGPIOx == GPIOH){
		GPIOH_REG_RESET();
	}
	else if(pGPIOx == GPIOI){
		GPIOI_REG_RESET();
	}
}
void GPIO_PClkControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi){
	if(EnorDi == ENABLE){
		if(pGPIOx == GPIOA){
			GPIOA_PCLK_EN();
		}
		else if(pGPIOx == GPIOB){
			GPIOB_PCLK_EN();
		}
		else if(pGPIOx == GPIOC){
				GPIOC_PCLK_EN();
		}
		else if(pGPIOx == GPIOD){
				GPIOD_PCLK_EN();
		}
		else if(pGPIOx == GPIOE){
				GPIOE_PCLK_EN();
		}
		else if(pGPIOx == GPIOF){
				GPIOF_PCLK_EN();
		}
		else if(pGPIOx == GPIOG){
				GPIOG_PCLK_EN();
		}
		else if(pGPIOx == GPIOH){
				GPIOH_PCLK_EN();
		}
		else if(pGPIOx == GPIOI){
				GPIOI_PCLK_EN();
		}
	}
	else{
		if(pGPIOx == GPIOA){
			GPIOA_PCLK_DI();
		}
		else if(pGPIOx == GPIOB){
			GPIOB_PCLK_DI();
		}
		else if(pGPIOx == GPIOC){
			GPIOC_PCLK_DI();
		}
		else if(pGPIOx == GPIOD){
			GPIOD_PCLK_DI();
		}
		else if(pGPIOx == GPIOE){
			GPIOE_PCLK_DI();
		}
		else if(pGPIOx == GPIOF){
			GPIOF_PCLK_DI();
		}
		else if(pGPIOx == GPIOG){
			GPIOG_PCLK_DI();
		}
		else if(pGPIOx == GPIOH){
			GPIOH_PCLK_DI();
		}
		else if(pGPIOx == GPIOI){
			GPIOI_PCLK_DI();
		}
	}
}

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){
	uint8_t val;
	val = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);
	return val;
}
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx){
	uint16_t val;
	val = (uint16_t)(pGPIOx->IDR);
	return val;
}
void GPIO_WriteFromOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t val){
	if(val == 1){
		pGPIOx->ODR |= (1<<PinNumber);
	}
	else{
		pGPIOx->ODR &= ~(1<<PinNumber);
	}
}
void GPIO_WriteFromOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t val){
	pGPIOx->ODR = val;
}
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){
	pGPIOx->ODR ^= (1<<PinNumber);
}
void GPIO_SetBSRR(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){
	pGPIOx->BSRR |= (1<<PinNumber);
}
void GPIO_ResetBSRR(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){
	PinNumber = PinNumber + 16;
	pGPIOx->BSRR &= ~(1<<PinNumber);
}
void GPIO_IRQITConfig(uint8_t IRQNum, uint8_t EnorDi){
	if(EnorDi == ENABLE){
		if(IRQNum <= 31){
			*NVIC_ISER0 |= (1<<IRQNum);
		}
		else if(IRQNum < 64){
			*NVIC_ISER1 |= ((1<<IRQNum%32));
		}
		else if(IRQNum < 96){
			*NVIC_ISER2 |= ((1<<IRQNum%32));
		}
	}
	else{
		if(IRQNum <= 31){
			*NVIC_ICER0 |= (1<<IRQNum);
		}
		else if(IRQNum < 64){
			*NVIC_ICER1 |= ((1<<IRQNum%32));
		}
		else if(IRQNum < 96){
			*NVIC_ICER2 |= ((1<<IRQNum%32));
		}
	}
}
void GPIO_IRQPriorityConfig(uint8_t IRQNum, uint32_t IRQPriority){
	uint8_t iprx = IRQNum / 4;
	uint8_t iprx_section = IRQNum % 4;
	uint8_t shamt = (8*iprx_section) + (8-NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BADDR + iprx) |= (IRQPriority << shamt);
}
void GPIO_IRQHandling(uint8_t PinNumber){
	if(EXTI->PR & (1<<PinNumber)){
		EXTI->PR |= (1 << PinNumber);
	}
}
