/*
 * stm32f4xx.h
 *
 *  Created on: Jan 26, 2024
 *      Author: PC
 */

#ifndef INC_STM32F4XX_H_
#define INC_STM32F4XX_H_

#include <stddef.h>
#include <stdint.h>

//PROCESSOR SPECIFIC

#define NVIC_ISER0					((volatile uint32_t*) 0xE000E100)
#define NVIC_ISER1					((volatile uint32_t*) 0xE000E104)
#define NVIC_ISER2					((volatile uint32_t*) 0xE000E108)
#define NVIC_ISER3					((volatile uint32_t*) 0xE000E10C)


#define NVIC_ICER0					((volatile uint32_t*) 0xE000E180)
#define NVIC_ICER1					((volatile uint32_t*) 0xE000E184)
#define NVIC_ICER2					((volatile uint32_t*) 0xE000E188)
#define NVIC_ICER3					((volatile uint32_t*) 0xE000E18C)

#define NO_PR_BITS_IMPLEMENTED		4

#define NVIC_PR_BADDR				((volatile uint32_t*) 0xE000E400)

#define FLASH_BADDR					0x08000000U	//flash memory base address
#define SRAM1_BADDR					0x20000000U //SRAM1 memory base address
#define SRAM2_BADDR					0x2001C000U //SRAM2 memory base address
#define ROM_BADDR					0x1FFF0000U //ROM (system memory) memory base address
#define SRAM_BADDR					SRAM1_BADDR //SRAM starts from sram1 base address



#define PERIPH_BADDR				0x40000000U
#define APB1_BADDR					PERIPH_BADDR
#define APB2_BADDR					0x40010000U
#define AHB1_BADDR					0x40020000U
#define AHB2_BADDR					0x50000000U

//AHB1 PERIPHERALS
#define GPIOA_BADDR					(AHB1_BADDR)
#define GPIOB_BADDR					(AHB1_BADDR + 0x0400)
#define GPIOC_BADDR					(AHB1_BADDR + 0x0800)
#define GPIOD_BADDR					(AHB1_BADDR + 0x0C00)
#define GPIOE_BADDR					(AHB1_BADDR + 0x1000)
#define GPIOF_BADDR					(AHB1_BADDR + 0x1400)
#define GPIOG_BADDR					(AHB1_BADDR + 0x1800)
#define GPIOH_BADDR					(AHB1_BADDR + 0x1C00)
#define GPIOI_BADDR					(AHB1_BADDR + 0x2000)

#define RCC_BADDR					(AHB1_BADDR + 0x3800)

//APB1 PERIPHERALS
#define TIM2_BADDR					(APB1_BADDR)
#define I2C1_BADDR					(APB1_BADDR + 0x5400)
#define I2C2_BADDR					(APB1_BADDR + 0x5800)
#define I2C3_BADDR					(APB1_BADDR + 0x5C00)

#define SPI2_BADDR					(APB1_BADDR + 0x3800)
#define SPI3_BADDR					(APB1_BADDR + 0x3C00)

#define USART2_BADDR				(APB1_BADDR + 0x4400)
#define USART3_BADDR				(APB1_BADDR + 0x4800)
#define UART4_BADDR					(APB1_BADDR + 0x4C00)
#define UART5_BADDR					(APB1_BADDR + 0x5000)

//APB2 PERIPHERALS
#define SPI1_BADDR					(APB2_BADDR + 0x3000)
#define SPI4_BADDR					(APB2_BADDR + 0x3400)

#define USART1_BADDR				(APB2_BADDR + 0x1000)
#define USART6_BADDR				(APB2_BADDR + 0x1400)

#define EXTI_BADDR					(APB2_BADDR + 0x3C00)
#define SYSCFG_BADDR				(APB2_BADDR + 0x3800)


//PERIPHERAL REGISTER DEFINITION STRUCTURE

typedef struct{
	volatile uint32_t MODER;
	volatile uint32_t OTYPER;
	volatile uint32_t OSPEEDR;
	volatile uint32_t PUPDR;
	volatile uint32_t IDR;
	volatile uint32_t ODR;
	volatile uint32_t BSRR;
	volatile uint32_t LCKR;
	volatile uint32_t AFR[2];
}GPIO_RegDef_t;

typedef struct{
	volatile uint32_t CR;
	volatile uint32_t PLLCFGR;
	volatile uint32_t CFGR;
	volatile uint32_t CIR;
	volatile uint32_t AHB1RSTR;
	volatile uint32_t AHB2RSTR;
	volatile uint32_t AHB3RSTR;
	uint32_t Reserved1;
	volatile uint32_t APB1RSTR;
	volatile uint32_t APB2RSTR;
	uint32_t Reserved2;
	uint32_t Reserved3;
	volatile uint32_t AHB1ENR;
	volatile uint32_t AHB2ENR;
	volatile uint32_t AHB3ENR;
	uint32_t Reserved4;
	volatile uint32_t APB1ENR;
	volatile uint32_t APB2ENR;
	uint32_t Reserved5;
	uint32_t Reserved6;
	volatile uint32_t AHB1LPENR;
	volatile uint32_t AHB2LPENR;
	volatile uint32_t AHB3LPENR;
	uint32_t Reserved7;
	volatile uint32_t APB1LPENR;
	volatile uint32_t APB2LPENR;
	uint32_t Reserved8;
	uint32_t Reserved9;
	volatile uint32_t BDCR;
	volatile uint32_t CSR;
	uint32_t Reserved10;
	uint32_t Reserved11;
	volatile uint32_t SSCGR;
	volatile uint32_t PLLI2SCFGR;
	volatile uint32_t PLLSAICFGR;
	volatile uint32_t DCKCFGR;
}RCC_RegDef_t;

typedef struct{
	volatile uint32_t IMR;
	volatile uint32_t EMR;
	volatile uint32_t RTSR;
	volatile uint32_t FTSR;
	volatile uint32_t SWIER;
	volatile uint32_t PR;
}EXTI_RegDef_t;

typedef struct{
	volatile uint32_t MEMRMP;
	volatile uint32_t PMC;
	volatile uint32_t EXTICR[4];
	uint32_t Reserved[2];
	volatile uint32_t CMPCR;
}SYSCFG_RegDef_t;

typedef struct{
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t SR;
	volatile uint32_t DR;
	volatile uint32_t CRCPR;
	volatile uint32_t RXCRCR;
	volatile uint32_t TXCRCR;
	volatile uint32_t I2SCFGR;
	volatile uint32_t I2SPR;
}SPI_RegDef_t;

typedef struct{
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t OAR1;
	volatile uint32_t OAR2;
	volatile uint32_t DR;
	volatile uint32_t SR1;
	volatile uint32_t SR2;
	volatile uint32_t CCR;
	volatile uint32_t TRISE;
	volatile uint32_t FLTR;
}I2C_RegDef_t;

typedef struct{
	volatile uint32_t SR;
	volatile uint32_t DR;
	volatile uint32_t BRR;
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t CR3;
	volatile uint32_t GTPR;
}USART_RegDef_t;

typedef struct{
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t SMCR;
	volatile uint32_t DIER;
	volatile uint32_t SR;
	volatile uint32_t EGR;
	volatile uint32_t CCMR1;
	volatile uint32_t CCMR2;
	volatile uint32_t CCER;
	volatile uint32_t CNT;
	volatile uint32_t PSC;
	volatile uint32_t ARR;
	uint32_t reserved1;
	volatile uint32_t CCR1;
	volatile uint32_t CCR2;
	volatile uint32_t CCR3;
	volatile uint32_t CCR4;
	uint32_t reserved2;
	volatile uint32_t DCR;
	volatile uint32_t DMAR;
	volatile uint32_t OR;
}TIM_RegDef_t;
//PERIPHERAL DEFINITIONS
#define TIM2  ((TIM_RegDef_t*) TIM2_BADDR)

#define GPIOA ((GPIO_RegDef_t*) GPIOA_BADDR)
#define GPIOB ((GPIO_RegDef_t*) GPIOB_BADDR)
#define GPIOC ((GPIO_RegDef_t*) GPIOC_BADDR)
#define GPIOD ((GPIO_RegDef_t*) GPIOD_BADDR)
#define GPIOE ((GPIO_RegDef_t*) GPIOE_BADDR)
#define GPIOF ((GPIO_RegDef_t*) GPIOF_BADDR)
#define GPIOG ((GPIO_RegDef_t*) GPIOG_BADDR)
#define GPIOH ((GPIO_RegDef_t*) GPIOH_BADDR)
#define GPIOI ((GPIO_RegDef_t*) GPIOI_BADDR)

#define RCC ((RCC_RegDef_t*) RCC_BADDR)
#define EXTI ((EXTI_RegDef_t*) EXTI_BADDR)
#define SYSCFG ((SYSCFG_RegDef_t*) SYSCFG_BADDR)

#define SPI1 ((SPI_RegDef_t*) SPI1_BADDR)
#define SPI2 ((SPI_RegDef_t*) SPI2_BADDR)
#define SPI3 ((SPI_RegDef_t*) SPI3_BADDR)
#define SPI4 ((SPI_RegDef_t*) SPI4_BADDR)

#define I2C1 ((I2C_RegDef_t*) I2C1_BADDR)
#define I2C2 ((I2C_RegDef_t*) I2C2_BADDR)
#define I2C3 ((I2C_RegDef_t*) I2C3_BADDR)

#define USART1 ((USART_RegDef_t*) USART1_BADDR)
#define USART2 ((USART_RegDef_t*) USART2_BADDR)
#define USART3 ((USART_RegDef_t*) USART3_BADDR)
#define UART4 ((USART_RegDef_t*) UART4_BADDR)
#define UART5 ((USART_RegDef_t*) UART5_BADDR)
#define USART6 ((USART_RegDef_t*) USART6_BADDR)

//CLOCK ENABLE FOR GPIOX PERIPHERALS
#define GPIOA_PCLK_EN()		(RCC->AHB1ENR |= (1<<0))
#define GPIOB_PCLK_EN()		(RCC->AHB1ENR |= (1<<1))
#define GPIOC_PCLK_EN()		(RCC->AHB1ENR |= (1<<2))
#define GPIOD_PCLK_EN()		(RCC->AHB1ENR |= (1<<3))
#define GPIOE_PCLK_EN()		(RCC->AHB1ENR |= (1<<4))
#define GPIOF_PCLK_EN()		(RCC->AHB1ENR |= (1<<5))
#define GPIOG_PCLK_EN()		(RCC->AHB1ENR |= (1<<6))
#define GPIOH_PCLK_EN()		(RCC->AHB1ENR |= (1<<7))
#define GPIOI_PCLK_EN()		(RCC->AHB1ENR |= (1<<8))

//CLOCK ENABLE FOR I2C PERIPHERALS
#define I2C1_PCLK_EN()		(RCC->APB1ENR |= (1<<21))
#define I2C2_PCLK_EN()		(RCC->APB1ENR |= (1<<22))
#define I2C3_PCLK_EN()		(RCC->APB1ENR |= (1<<23))

//CLOCK ENABLE FOR SPI PERIPHERALS
#define SPI1_PCLK_EN()		(RCC->APB2ENR |= (1<<12))
#define SPI2_PCLK_EN()		(RCC->APB1ENR |= (1<<14))
#define SPI3_PCLK_EN()		(RCC->APB1ENR |= (1<<15))
#define SPI4_PCLK_EN()		(RCC->APB2ENR |= (1<<13))

//CLOCK ENABLE FOR USART PERIPHERALS
#define USART1_PCLK_EN()	(RCC->APB2ENR |= (1<<4))
#define USART2_PCLK_EN()	(RCC->APB1ENR |= (1<<17))
#define USART3_PCLK_EN()	(RCC->APB1ENR |= (1<<18))
#define UART4_PCLK_EN()		(RCC->APB1ENR |= (1<<19))
#define UART5_PCLK_EN()		(RCC->APB1ENR |= (1<<20))
#define USART6_PCLK_EN()	(RCC->APB2ENR |= (1<<5))

//SYSCFG ENABLE
#define SYSCFG_PCLK_EN()	(RCC->APB2ENR |= (1<<14))

//CLOCK DISABLE FOR GPIOX PERIPHERALS
#define GPIOA_PCLK_DI()		(RCC->AHB1ENR &= ~(1<<0))
#define GPIOB_PCLK_DI()		(RCC->AHB1ENR &= ~(1<<1))
#define GPIOC_PCLK_DI()		(RCC->AHB1ENR &= ~(1<<2))
#define GPIOD_PCLK_DI()		(RCC->AHB1ENR &= ~(1<<3))
#define GPIOE_PCLK_DI()		(RCC->AHB1ENR &= ~(1<<4))
#define GPIOF_PCLK_DI()		(RCC->AHB1ENR &= ~(1<<5))
#define GPIOG_PCLK_DI()		(RCC->AHB1ENR &= ~(1<<6))
#define GPIOH_PCLK_DI()		(RCC->AHB1ENR &= ~(1<<7))
#define GPIOI_PCLK_DI()		(RCC->AHB1ENR &= ~(1<<8))

//CLOCK DISABLE FOR I2C PERIPHERALS
#define I2C1_PCLK_DI()		(RCC->APB1ENR &= ~(1<<21))
#define I2C2_PCLK_DI()		(RCC->APB1ENR &= ~(1<<22))
#define I2C3_PCLK_DI()		(RCC->APB1ENR &= ~(1<<23))

//CLOCK DISABLE FOR SPI PERIPHERALS
#define SPI1_PCLK_DI()		(RCC->APB2ENR &= ~(1<<12))
#define SPI2_PCLK_DI()		(RCC->APB1ENR &= ~(1<<14))
#define SPI3_PCLK_DI()		(RCC->APB1ENR &= ~(1<<15))
#define SPI4_PCLK_DI()		(RCC->APB2ENR &= ~(1<<13))

//CLOCK DISABLE FOR USART PERIPHERALS
#define USART1_PCLK_DI()	(RCC->APB2ENR &= ~(1<<4))
#define USART2_PCLK_DI()	(RCC->APB1ENR &= ~(1<<17))
#define USART3_PCLK_DI()	(RCC->APB1ENR &= ~(1<<18))
#define UART4_PCLK_DI()		(RCC->APB1ENR &= ~(1<<19))
#define UART5_PCLK_DI()		(RCC->APB1ENR &= ~(1<<20))
#define USART6_PCLK_DI()	(RCC->APB2ENR &= ~(1<<5))

//SYSCFG DISABLE
#define SYSCFG_PCLK_DI()	(RCC->APB2ENR &= ~(1<<14))

//RESET GPIOx PERIPHERALS
#define GPIOA_REG_RESET() do{(RCC->AHB1RSTR |= (1<<0)); (RCC->AHB1RSTR &= ~(1<<0));}while(0)
#define GPIOB_REG_RESET() do{(RCC->AHB1RSTR |= (1<<1)); (RCC->AHB1RSTR &= ~(1<<1));}while(0)
#define GPIOC_REG_RESET() do{(RCC->AHB1RSTR |= (1<<2)); (RCC->AHB1RSTR &= ~(1<<2));}while(0)
#define GPIOD_REG_RESET() do{(RCC->AHB1RSTR |= (1<<3)); (RCC->AHB1RSTR &= ~(1<<3));}while(0)
#define GPIOE_REG_RESET() do{(RCC->AHB1RSTR |= (1<<4)); (RCC->AHB1RSTR &= ~(1<<4));}while(0)
#define GPIOF_REG_RESET() do{(RCC->AHB1RSTR |= (1<<5)); (RCC->AHB1RSTR &= ~(1<<5));}while(0)
#define GPIOG_REG_RESET() do{(RCC->AHB1RSTR |= (1<<6)); (RCC->AHB1RSTR &= ~(1<<6));}while(0)
#define GPIOH_REG_RESET() do{(RCC->AHB1RSTR |= (1<<7)); (RCC->AHB1RSTR &= ~(1<<7));}while(0)
#define GPIOI_REG_RESET() do{(RCC->AHB1RSTR |= (1<<8)); (RCC->AHB1RSTR &= ~(1<<8));}while(0)

#define GPIO_BASE_TO_CODE(x) (((uint32_t)(x)-AHB1_BADDR)/0x0400)

//RESET SPIX PERIPHERALS
#define SPI1_REG_RESET() do{(RCC->APB2RSTR |= (1<<12)); (RCC->APB2RSTR &= ~(1<<12));}while(0)
#define SPI2_REG_RESET() do{(RCC->APB1RSTR |= (1<<14)); (RCC->APB1RSTR &= ~(1<<14));}while(0)
#define SPI3_REG_RESET() do{(RCC->APB1RSTR |= (1<<15)); (RCC->APB1RSTR &= ~(1<<15));}while(0)
#define SPI4_REG_RESET() do{(RCC->APB2RSTR |= (1<<13)); (RCC->APB2RSTR &= ~(1<<13));}while(0)

//RESET I2Cx PERIPHERALS
#define I2C1_REG_RESET() do{(RCC->APB1RSTR |= (1<<21)); (RCC->APB1RSTR &= ~(1<<21));}while(0)
#define I2C2_REG_RESET() do{(RCC->APB1RSTR |= (1<<22)); (RCC->APB1RSTR &= ~(1<<22));}while(0)
#define I2C3_REG_RESET() do{(RCC->APB1RSTR |= (1<<23)); (RCC->APB1RSTR &= ~(1<<23));}while(0)

//RESET USART PERIPHERALS
#define USART1_REG_RESET() do{(RCC->APB2RSTR |= (1<<4)); (RCC->APB2RSTR &= ~(1<<4));}while(0)
#define USART2_REG_RESET() do{(RCC->APB1RSTR |= (1<<17)); (RCC->APB1RSTR &= ~(1<<17));}while(0)
#define USART3_REG_RESET() do{(RCC->APB1RSTR |= (1<<18)); (RCC->APB1RSTR &= ~(1<<18));}while(0)
#define UART4_REG_RESET() do{(RCC->APB1RSTR |= (1<<19)); (RCC->APB1RSTR &= ~(1<<19));}while(0)
#define UART5_REG_RESET() do{(RCC->APB1RSTR |= (1<<20)); (RCC->APB1RSTR &= ~(1<<20));}while(0)
#define USART6_REG_RESET() do{(RCC->APB2RSTR |= (1<<5)); (RCC->APB2RSTR &= ~(1<<5));}while(0)


#define IRQ_NO_EXTI0		6
#define IRQ_NO_EXTI1		7
#define IRQ_NO_EXTI2		8
#define IRQ_NO_EXTI3		9
#define IRQ_NO_EXTI4		10
#define IRQ_NO_EXTI9_5		23
#define IRQ_NO_EXTI10_15	40
#define IRQ_NO_SPI1			35
#define IRQ_NO_SPI2			36
#define IRQ_NO_SPI3			51
#define IRQ_NO_I2C1_EV		31
#define IRQ_NO_I2C1_ERR		32

#define ENABLE 1
#define DISABLE 0
#define SET ENABLE
#define RESET DISABLE


#include "stm32f4xx_gpio.h"
#include "stm32f4xx_spi.h"
#include "stm32f4xx_i2c.h"
#include "stm32f4xx_usart.h"
#endif /* INC_STM32F4XX_H_ */
