/*
 * stm32f4xx_spi.c
 *
 *  Created on: Jan 31, 2024
 *      Author: PC
 */
#include "stm32f4xx_spi.h"

static void SPI_TXE_IT_Handle(SPI_Handle_t *pSPIHandle);
static void SPI_RXNE_IT_Handle(SPI_Handle_t *pSPIHandle);
static void SPI_OVR_ERR_Handle(SPI_Handle_t *pSPIHandle);

void SPI_Init(SPI_Handle_t *pSPIHandle) {
	uint32_t tempreg = 0;

	SPI_PClkControl(pSPIHandle->pSPIx, ENABLE);

	tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << 2;

	if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CFG_FD) {
		tempreg &= ~(1 << 15);
	} else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CFG_HD) {
		tempreg |= (1 << 15);
	} else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CFG_SMPL_RXONLY) {
		tempreg &= ~(1 << 15);
		tempreg |= (1 << 10);
	}

	//speed
	tempreg |= (pSPIHandle->SPIConfig.SPI_SClkSpeed << 3);

	//DFF
	tempreg |= (pSPIHandle->SPIConfig.SPI_DFF << 11);

	//CPOL
	tempreg |= (pSPIHandle->SPIConfig.SPI_CPOL << 1);
	//CPHA
	tempreg |= (pSPIHandle->SPIConfig.SPI_CPHA << 0);
	//SSM
	tempreg |= (pSPIHandle->SPIConfig.SPI_SSM << 9);

	pSPIHandle->pSPIx->CR1 = tempreg;
}
void SPI_DeInit(SPI_RegDef_t *pSPIx) {
	if (pSPIx == SPI1) {
		SPI1_REG_RESET();
	} else if (pSPIx == SPI2) {
		SPI2_REG_RESET();
	} else if (pSPIx == SPI3) {
		SPI3_REG_RESET();
	} else if (pSPIx == SPI4) {
		SPI4_REG_RESET();
	}
}
void SPI_PClkControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi) {
	if (EnorDi == ENABLE) {
		if (pSPIx == SPI1) {
			SPI1_PCLK_EN();
		} else if (pSPIx == SPI2) {
			SPI2_PCLK_EN();
		} else if (pSPIx == SPI3) {
			SPI3_PCLK_EN();
		} else if (pSPIx == SPI4) {
			SPI4_PCLK_EN();
		}
	} else {
		if (pSPIx == SPI1) {
			SPI1_PCLK_DI();
		} else if (pSPIx == SPI2) {
			SPI2_PCLK_DI();
		} else if (pSPIx == SPI3) {
			SPI3_PCLK_DI();
		} else if (pSPIx == SPI4) {
			SPI4_PCLK_DI();
		}
	}
}

//send and recieve
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len) {
	while (Len > 0) {
		 while (!((pSPIx->SR)&(1<<1)))
			; //wait until tx buffer ready
		if (pSPIx->CR1 & (1 << 11)) {
			//16 Bit dff
			pSPIx->DR = *((uint16_t*) pTxBuffer);
			Len--;
			Len--;
			(uint16_t*) pTxBuffer++;
		} else {
			//8bit dff
			pSPIx->DR = *(pTxBuffer);
			Len--;
			pTxBuffer++;
		}
	}
}
void SPI_RecieveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len) {
	while (Len > 0) {
		while (!(pSPIx->SR & (1 << 0)))
			; //wait until rx buffer ready
		if (pSPIx->CR1 & (1 << 11)) {
			//16 Bit dff
			*((uint16_t*) pRxBuffer) = pSPIx->DR;
			Len--;
			Len--;
			(uint16_t*) pRxBuffer++;
		} else {
			//8bit dff
			*(pRxBuffer) = pSPIx->DR;
			Len--;
			pRxBuffer++;
		}
	}
}

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer,
		uint32_t Len) {
	uint8_t state = pSPIHandle->TxState;

	if (state != SPI_BUSY_IN_TX) {
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = Len;

		pSPIHandle->TxState = SPI_BUSY_IN_TX;

		pSPIHandle->pSPIx->CR2 |= (1 << 7);
	}
	return state;
}
uint8_t SPI_RecieveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer,
		uint32_t Len) {
	uint8_t state = pSPIHandle->RxState;

	if (state != SPI_BUSY_IN_RX) {
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = Len;

		pSPIHandle->RxState = SPI_BUSY_IN_RX;

		pSPIHandle->pSPIx->CR2 |= (1 << 6);
	}
	return state;
}
//irq config
void SPI_IRQITConfig(uint8_t IRQNum, uint8_t EnorDi) {
	if (EnorDi == ENABLE) {
		if (IRQNum <= 31) {
			*NVIC_ISER0 |= (1 << IRQNum);
		} else if (IRQNum < 64) {
			*NVIC_ISER1 |= ((1 << IRQNum % 32));
		} else if (IRQNum < 96) {
			*NVIC_ISER2 |= ((1 << IRQNum % 32));
		}
	} else {
		if (IRQNum <= 31) {
			*NVIC_ICER0 |= (1 << IRQNum);
		} else if (IRQNum < 64) {
			*NVIC_ICER1 |= ((1 << IRQNum % 32));
		} else if (IRQNum < 96) {
			*NVIC_ICER2 |= ((1 << IRQNum % 32));
		}
	}
}
void SPI_IRQPriorityConfig(uint8_t IRQNum, uint32_t IRQPriority) {
	uint8_t iprx = IRQNum / 4;
	uint8_t iprx_section = IRQNum % 4;
	uint8_t shamt = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BADDR + iprx) |= (IRQPriority << shamt);
}
void SPI_IRQHandling(SPI_Handle_t *pHandle) {
	uint8_t temp1, temp2;
	temp1 = pHandle->pSPIx->SR & (1 << 1);
	temp2 = pHandle->pSPIx->CR2 & (1 << 7);

	if (temp1 && temp2) {
		SPI_TXE_IT_Handle(pHandle);
	}
	temp1 = pHandle->pSPIx->SR & (1 << 0);
	temp2 = pHandle->pSPIx->CR2 & (1 << 6);

	if (temp1 && temp2) {
		SPI_RXNE_IT_Handle(pHandle);
	}

	temp1 = pHandle->pSPIx->SR & (1 << 6);
	temp2 = pHandle->pSPIx->CR2 & (1 << 5);
	if (temp1 && temp2) {
		SPI_OVR_ERR_Handle(pHandle);
	}
}

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi) {
	if (EnorDi == ENABLE) {
		pSPIx->CR1 |= (1 << 6);
	} else {
		pSPIx->CR1 &= ~(1 << 6);
	}
}
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi) {
	if (EnorDi == ENABLE) {
		pSPIx->CR1 |= (1 << 8);
	} else {
		pSPIx->CR1 &= ~(1 << 8);
	}
}

void  SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR2 |=  (1 << 2);
	}else
	{
		pSPIx->CR2 &=  ~(1 << 2);
	}
}
static void SPI_TXE_IT_Handle(SPI_Handle_t *pSPIHandle) {
	if (pSPIHandle->pSPIx->CR1 & (1 << 11)) {
		//16 Bit dff
		pSPIHandle->pSPIx->DR = *((uint16_t*) pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen--;
		pSPIHandle->TxLen--;
		(uint16_t*) pSPIHandle->pTxBuffer++;
	} else {
		//8bit dff
		pSPIHandle->pSPIx->DR = *(pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen--;
		pSPIHandle->pTxBuffer++;
	}

	if (!(pSPIHandle->TxLen)) {
		pSPIHandle->pSPIx->CR2 &= ~(1 << 7);
		pSPIHandle->pTxBuffer = NULL;
		pSPIHandle->TxLen = 0;
		pSPIHandle->TxState = SPI_READY;
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);
	}
}
static void SPI_RXNE_IT_Handle(SPI_Handle_t *pSPIHandle) {
	if (pSPIHandle->pSPIx->CR1 & (1 << 11)) {
		//16 Bit dff
		*((uint16_t*) pSPIHandle->pRxBuffer) = (uint16_t)pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen--;
		pSPIHandle->RxLen--;
		(uint16_t*) pSPIHandle->pRxBuffer++;
	} else {
		//8bit dff
		*(pSPIHandle->pRxBuffer) = (uint8_t)pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen--;
		pSPIHandle->pRxBuffer++;
	}

	if (!(pSPIHandle->RxLen)) {
		pSPIHandle->pSPIx->CR2 &= ~(1 << 6);
		pSPIHandle->pRxBuffer = NULL;
		pSPIHandle->RxLen = 0;
		pSPIHandle->RxState = SPI_READY;
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);
	}
}
static void SPI_OVR_ERR_Handle(SPI_Handle_t *pSPIHandle){

}
