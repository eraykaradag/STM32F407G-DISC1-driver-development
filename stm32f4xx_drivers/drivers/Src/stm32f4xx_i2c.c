/*
 * stm32f4xx_i2c.c
 *
 *  Created on: Feb 4, 2024
 *      Author: PC
 */

#include "stm32f4xx_i2c.h"
uint32_t RCC_GetPLLOutputClock(void) {
	return 0;
}
static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx) {
	pI2Cx->CR1 |= (1 << 8);
}
static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx) {
	pI2Cx->CR1 |= (1 << 9);
}
static void I2C_ExecuteAddresPhase(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr,
		uint8_t RW) {
	SlaveAddr = SlaveAddr << 1;
	if (RW == 0) { //SENDING DATA
		SlaveAddr &= ~(1);
	} else { //RECIEVING DATA
		SlaveAddr |= 1;
	}
	pI2Cx->DR = SlaveAddr;
}
static void I2C_ClearAddrFlag(I2C_Handle_t *pI2CHandle) {
	uint32_t dummy;
	if (pI2CHandle->pI2Cx->SR2 & (1 << 0)) {
		//master mode
		if (pI2CHandle->TxRxState == I2C_BUSY_RX) {
			if (pI2CHandle->RxSize == 1) {
				I2C_ManageAck(pI2CHandle->pI2Cx, DISABLE);
				dummy = pI2CHandle->pI2Cx->SR1;
				dummy = pI2CHandle->pI2Cx->SR2;
				(void) dummy;
			}
		} else {
			dummy = pI2CHandle->pI2Cx->SR1;
			dummy = pI2CHandle->pI2Cx->SR2;
			(void) dummy;
		}
	} else {
		//slave mode
		dummy = pI2CHandle->pI2Cx->SR1;
		dummy = pI2CHandle->pI2Cx->SR2;
		(void) dummy;
	}
}
uint16_t AHB_PreScaler[9] = { 2, 4, 8, 16, 32, 64, 128, 256, 512 };
uint8_t APB1_PreScaler[4] = { 2, 4, 8, 16 };
uint32_t RCC_GetPCLK1Val(void) {
	uint32_t pclk1, SystemClk;
	uint8_t clcksrc = ((RCC->CFGR >> 2) & 0x3);

	if (clcksrc == 0) {
		//HSI
		SystemClk = 16000000;
	} else if (clcksrc == 1) {
		//HSE
		SystemClk = 8000000;
	} else if (clcksrc == 2) {
		//PLL
		SystemClk = RCC_GetPLLOutputClock();
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
void I2C_PClkControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi) {
	if (EnorDi == ENABLE) {
		if (pI2Cx == I2C1) {
			I2C1_PCLK_EN();
		} else if (pI2Cx == I2C2) {
			I2C2_PCLK_EN();
		} else if (pI2Cx == I2C3) {
			I2C3_PCLK_EN();
		}
	} else {
		if (pI2Cx == I2C1) {
			I2C1_PCLK_DI();
		} else if (pI2Cx == I2C2) {
			I2C2_PCLK_DI();
		} else if (pI2Cx == I2C3) {
			I2C3_PCLK_DI();
		}
	}
}

void I2C_Init(I2C_Handle_t *pI2CHandle) {
	uint32_t temp = 0;

	//I2C PERIPH CLOCK CONTROL
	I2C_PClkControl(pI2CHandle->pI2Cx, ENABLE);

	temp |= pI2CHandle->I2C_Config.I2C_AckCtrl << 10;
	pI2CHandle->pI2Cx->CR1 = temp;

	//FREQ
	temp = 0;
	temp |= RCC_GetPCLK1Val() / 1000000U;
	pI2CHandle->pI2Cx->CR2 = (temp & 0x3F);

	temp = pI2CHandle->I2C_Config.I2C_DeviceAddr << 1;
	temp |= (1 << 14);
	pI2CHandle->pI2Cx->OAR1 = temp;

	//CCR calc
	uint16_t ccrval = 0;
	temp = 0;
	if (pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM) {
		//standard mode
		ccrval =
				(RCC_GetPCLK1Val() / (2 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		temp |= (ccrval & 0xFFF);
	} else {
		//fast mode
		temp |= (1 << 15);
		temp |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14);
		if (pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2) {
			ccrval = (RCC_GetPCLK1Val()
					/ (3 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		} else if (pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_16_9) {
			ccrval = (RCC_GetPCLK1Val()
					/ (25 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		}
		temp |= (ccrval & 0xFFF);
	}
	pI2CHandle->pI2Cx->CCR = temp;

	//trise
	if (pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM) {
		//standard mode
		temp = (RCC_GetPCLK1Val() / 1000000U) + 1;
	} else {
		temp = ((RCC_GetPCLK1Val() * 300) / 1000000U) + 1;
	}
	pI2CHandle->pI2Cx->TRISE = (temp & 0x3F);
}
void I2C_DeInit(I2C_RegDef_t *pI2Cx) {
	if (pI2Cx == I2C1) {
		I2C1_REG_RESET();
	} else if (pI2Cx == I2C2) {
		I2C2_REG_RESET();
	} else if (pI2Cx == I2C3) {
		I2C3_REG_RESET();
	}
}

//send data
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer,
		uint32_t Len, uint8_t SlaveAddr) {
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//wait until sb == 1
	while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB))
		;

	I2C_ExecuteAddresPhase(pI2CHandle->pI2Cx, SlaveAddr, 0);

	while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR))
		;

	I2C_ClearAddrFlag(pI2CHandle);

	while (Len > 0) {
		while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE))
			;

		pI2CHandle->pI2Cx->DR = *pTxBuffer;
		pTxBuffer++;
		Len--;
	}
	while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE))
		;
	while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BTF))
		;

	I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
}
// master recieve data
void I2C_MasterRecieveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer,
		uint32_t Len, uint8_t SlaveAddr) {
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);
	while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB))
		;
	I2C_ExecuteAddresPhase(pI2CHandle->pI2Cx, SlaveAddr, 1);
	while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR))
		;

	if (Len == 1) {
		//Disable ACK
		I2C_ManageAck(pI2CHandle->pI2Cx, DISABLE);
		//clear addr flag
		I2C_ClearAddrFlag(pI2CHandle);
		//wait until rxne becomes 1
		while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE))
			;
		//generate stop condition
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
		//read data into buffer
		*pRxBuffer = pI2CHandle->pI2Cx->DR;
	}
	if (Len > 1) {
		//clear addr flag
		I2C_ClearAddrFlag(pI2CHandle);
		for (uint32_t i = Len; i > 0; i--) {
			//wait until rxne becomes 1
			while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE))
				;

			if (i == 2) {
				//cler ack bit
				I2C_ManageAck(pI2CHandle->pI2Cx, DISABLE);
				//generate stop condition
				I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
			}
			*pRxBuffer = pI2CHandle->pI2Cx->DR;
			pRxBuffer++;
		}
	}
	if (pI2CHandle->I2C_Config.I2C_AckCtrl == ENABLE) {
		I2C_ManageAck(pI2CHandle->pI2Cx, ENABLE);
	}

}
//irq config
void I2C_IRQITConfig(uint8_t IRQNum, uint8_t EnorDi) {
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

void I2C_IRQPriorityConfig(uint8_t IRQNum, uint32_t IRQPriority) {
	uint8_t iprx = IRQNum / 4;
	uint8_t iprx_section = IRQNum % 4;
	uint8_t shamt = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BADDR + iprx) |= (IRQPriority << shamt);
}

void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi) {
	if (EnorDi == ENABLE) {
		pI2Cx->CR1 |= (1 << 0);
	} else {
		pI2Cx->CR1 &= ~(1 << 0);
	}
}
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t flagName) {
	if (pI2Cx->SR1 & flagName) {
		return 1;
	} else {
		return 0;
	}
}
void I2C_ManageAck(I2C_RegDef_t *pI2Cx, uint8_t EnorDi) {
	if (EnorDi == ENABLE) {
		pI2Cx->CR1 |= (1 << 10);
	} else {
		pI2Cx->CR1 &= ~(1 << 10);
	}
}

void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv) {

}
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer,
		uint32_t Len, uint8_t SlaveAddr, uint8_t Sr) {

	uint8_t busystate = pI2CHandle->TxRxState;

	if ((busystate != I2C_BUSY_TX) && (busystate != I2C_BUSY_RX)) {
		pI2CHandle->pTxBuffer = pTxBuffer;
		pI2CHandle->TxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_TX;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		//Implement code to Generate START Condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);
		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (1 << 10);

		//Implement the code to enable ITEVFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (1 << 9);
		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (1 << 8);
	}

	return busystate;
}
uint8_t I2C_MasterRecieveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer,
		uint32_t Len, uint8_t SlaveAddr, uint8_t Sr) {
	uint8_t busystate = pI2CHandle->TxRxState;

	if ((busystate != I2C_BUSY_TX) && (busystate != I2C_BUSY_RX)) {
		pI2CHandle->pRxBuffer = pRxBuffer;
		pI2CHandle->RxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_RX;
		pI2CHandle->RxSize = Len; //Rxsize is used in the ISR code to manage the data reception
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		//Implement code to Generate START Condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);
		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (1 << 10);
		//Implement the code to enable ITEVFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (1 << 9);
		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (1 << 8);
	}

	return busystate;
}
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle) {
	uint32_t temp1, temp2, temp3;
	temp1 = pI2CHandle->pI2Cx->CR2 & (1 << 9);
	temp2 = pI2CHandle->pI2Cx->CR2 & (1 << 10);

	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << 0);
	if (temp1 && temp3) {
		//startcondition generated we should execute adress phase depending on the state
		if (pI2CHandle->TxRxState == I2C_BUSY_TX) {
			I2C_ExecuteAddresPhase(pI2CHandle->pI2Cx, pI2CHandle->DevAddr, 0);
		} else if (pI2CHandle->TxRxState == I2C_BUSY_RX) {
			I2C_ExecuteAddresPhase(pI2CHandle->pI2Cx, pI2CHandle->DevAddr, 1);
		}
	}
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << 1);
	if (temp1 && temp3) {
		//ADDR
		I2C_ClearAddrFlag(pI2CHandle);
	}
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << 2);
	if (temp1 && temp3) {
		//BTF
		if (pI2CHandle->TxRxState == I2C_BUSY_TX) {
			if (pI2CHandle->pI2Cx->SR1 & (1 << 7)) {
				if (pI2CHandle->TxLen == 0) {
					//BTF and TXE both set so we should close the transmission
					if (pI2CHandle->Sr == I2C_SR_DISABLE) {
						I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
					}
					//RESET EVERYTHING
					I2C_CloseTransmission(pI2CHandle);
					I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_TX_CMPLT);
				}

			}
		} else if (pI2CHandle->TxRxState == I2C_BUSY_RX) {

		}
	}
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << 4);
	if (temp1 && temp3) {
		//STOPF
		//happens in slave mode when it's set we should read SR1 and write to CR1 but we should write just 0s
		pI2CHandle->pI2Cx->CR1 |= 0x0000;
		//notify the app that stop generated by the master
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_STOP);
	}
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << 7);
	if (temp1 && temp2 && temp3) {
		//TXE
		if (pI2CHandle->pI2Cx->SR2 & (1 << 0)) {
			if (pI2CHandle->TxRxState == I2C_BUSY_TX) {
				if (pI2CHandle->TxLen > 0) {
					pI2CHandle->pI2Cx->DR = *(pI2CHandle->pTxBuffer);
					pI2CHandle->TxLen--;
					pI2CHandle->pTxBuffer++;
				}
			}
		}

	}
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << 6);
	if (temp1 && temp2 && temp3) {
		//RXNE
		if (pI2CHandle->pI2Cx->SR2 & (1 << 0)) {
			if (pI2CHandle->TxRxState == I2C_BUSY_RX) {
				if (pI2CHandle->RxSize == 1) {
					*(pI2CHandle->pRxBuffer) = pI2CHandle->pI2Cx->DR;
					pI2CHandle->RxLen--;
				} else if (pI2CHandle->RxSize > 1) {
					if (pI2CHandle->RxLen == 2) {
						I2C_ManageAck(pI2CHandle->pI2Cx, DISABLE);
					}
					*(pI2CHandle->pRxBuffer) = pI2CHandle->pI2Cx->DR;
					pI2CHandle->pRxBuffer++;
					pI2CHandle->RxLen--;
				}
				if (pI2CHandle->RxLen == 0) {
					//close and notify
					//generate stop condition if it's not repeated start
					if (pI2CHandle->Sr == I2C_SR_DISABLE) {
						I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
					}
					//CLOSE
					I2C_CloseRecieving(pI2CHandle);

					//notify app
					I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_RX_CMPLT);
				}
			}
		}

	}
}
void I2C_CloseRecieving(I2C_Handle_t *pI2CHandle){
	pI2CHandle->pI2Cx->CR2 &= ~(1 << 10);
	pI2CHandle->pI2Cx->CR2 &= ~(1 << 9);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pRxBuffer = NULL;
	pI2CHandle->RxSize = 0;
	pI2CHandle->RxLen = 0;
	if(pI2CHandle->I2C_Config.I2C_AckCtrl == I2C_ACK_EN)
		I2C_ManageAck(pI2CHandle->pI2Cx, ENABLE);
}
void I2C_CloseTransmission(I2C_Handle_t *pI2CHandle){
	pI2CHandle->pI2Cx->CR2 &= ~(1 << 10);
	pI2CHandle->pI2Cx->CR2 &= ~(1 << 9);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pTxBuffer = NULL;
	pI2CHandle->TxLen = 0;

}
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle) {
	uint32_t temp1,temp2;

	    //Know the status of  ITERREN control bit in the CR2
		temp2 = (pI2CHandle->pI2Cx->CR2) & ( 1 << 8);


	/***********************Check for Bus error************************************/
		temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1<< 8);
		if(temp1  && temp2 )
		{
			//This is Bus error

			//Implement the code to clear the buss error flag
			pI2CHandle->pI2Cx->SR1 &= ~( 1 << 8);

			//Implement the code to notify the application about the error
		   I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_BERR);
		}

	/***********************Check for arbitration lost error************************************/
		temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << 9 );
		if(temp1  && temp2)
		{
			//This is arbitration lost error

			//Implement the code to clear the arbitration lost error flag
			pI2CHandle->pI2Cx->SR1 &= ~( 1 << 9);
			//Implement the code to notify the application about the error
			I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_ARLO);
		}

	/***********************Check for ACK failure  error************************************/

		temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << 10);
		if(temp1  && temp2)
		{
			//This is ACK failure error

		    //Implement the code to clear the ACK failure error flag
			pI2CHandle->pI2Cx->SR1 &= ~( 1 << 10);
			//Implement the code to notify the application about the error
			I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_AF);
		}

	/***********************Check for Overrun/underrun error************************************/
		temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << 11);
		if(temp1  && temp2)
		{
			//This is Overrun/underrun

		    //Implement the code to clear the Overrun/underrun error flag
			pI2CHandle->pI2Cx->SR1 &= ~( 1 << 11);
			//Implement the code to notify the application about the error
			I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_OVR);
		}

	/***********************Check for Time out error************************************/
		temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << 14);
		if(temp1  && temp2)
		{
			//This is Time out error

		    //Implement the code to clear the Time out error flag
			pI2CHandle->pI2Cx->SR1 &= ~( 1 << 14);
			//Implement the code to notify the application about the error
			I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_TIMEOUT);
		}
}
