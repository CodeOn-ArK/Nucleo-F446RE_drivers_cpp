/*
 * stm32f446xx_spi.c
 *
 *  Created on: Feb 1, 2021
 *      Author: ark
 */

#include "stm32f446xx_spi.h"

static void SPI_RXNE_IT_Handle(SPI_Handle_t *pSPIHandle);
static void SPI_TXE_IT_Handle(SPI_Handle_t *pSPIHandle);
static void SPI_OVR_IT_Handle(SPI_Handle_t *pSPIHandle);

uint8_t SPI_GetFagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
	if(pSPIx->SR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

void SPI_Enable(SPI_RegDef_t *SPI_ENDI, uint8_t EN_DI)
{
	if(EN_DI == ENABLE)
	{
		SPI_ENDI->CR1 |=  (0x1 << SPIEN);
	}else
	{
		SPI_ENDI->CR1 &= ~(0x1 << SPIEN);
	}
}

void SSI_Config(SPI_RegDef_t *SPI_ENDI, uint8_t EN_DI)
{
	//SSI INITS -- THIS MAKES NSS PIN HIGH  AND PREVENTS MODF ERR -- NOT REQD FOR H/W SLAVE MANAGEMENT

	if(EN_DI == ENABLE)
	{
		SPI_ENDI->CR1 |=  (0x1 << SSIEN);
	}else
	{
		SPI_ENDI->CR1 &= ~(0x1 << SSIEN);
	}

}

void SSOE_Config(SPI_RegDef_t *SPI_ENDI, uint8_t EN_DI)
{

	if(EN_DI == ENABLE)
	{
		SPI_ENDI->CR2 |=  (0x1 << SSOEEN);
	}else
	{
		SPI_ENDI->CR2 &= ~(0x1 << SSOEEN);
	}
}
/********************************************************************************
 * 							DOCUMENTATION
 * @fn		:
 *
 * @brief	:
 *
 * @param	:
 * @param	:
 * @param	:
 *
 * @return  :
 *
 * @Note	:
 */
void SPI_PeriClkCntrl(SPI_RegDef_t *pSPIx, uint8_t En_Di)
{

	if(En_Di == ENABLE)
	{
		if(pSPIx == SPI1)
		{
			SPI1_CLK_EN();
		}
		else if(pSPIx == SPI2)
		{
			SPI2_CLK_EN();
		}
		else if(pSPIx == SPI3)
		{
			SPI3_CLK_EN();
		}
		else if(pSPIx == SPI4)
		{
			SPI4_CLK_EN();
		}

	}

	else if(En_Di == DISABLE)
	{
		if(pSPIx == SPI1)
		{
			SPI1_CLK_DI();
		}
		else if(pSPIx == SPI2)
		{
			SPI2_CLK_DI();
		}
		else if(pSPIx == SPI3)
		{
			SPI3_CLK_DI();
		}
		else if(pSPIx == SPI4)
		{
			SPI4_CLK_DI();
		}

	}
}

/********************************************************************************
 * 							DOCUMENTATION
 * @fn		:
 *
 * @brief	:
 *
 * @param	:
 * @param	:
 * @param	:
 *
 * @return  :
 *
 * @Note	:
 */
void SPI_Init(SPI_Handle_t *pSPIHandle)									//INITIALIZES SPI PORT
{

	//ENABLE CLOCK FOR THE SPI PERIPHERAL
	SPI_PeriClkCntrl(pSPIHandle->pSPIx, ENABLE);

	//FIRST CONFIGURE THE SPI_CR1 REG

	uint32_t temp = 0;

	//CONFIGURE THE DEVICE MODE
	temp = (pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR);

	//CONFIGURE THE BUS MODE
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		//BIDIMODE IS CLEARED TO ENABLE BIDIRECTIONAL MODE
		temp &= ~(1 << SPI_CR1_BIDIMODE);

	}else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		//BIDIMODE IS SET TO DISABLE BIDIRECTIONAL MODE
		temp |= (1 << SPI_CR1_BIDIOE);

	}else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_Rx)
	{
		//BIDIMODE IS CLEARED TO ENABLE BIDIRECTIONAL MODE
		temp &= ~(1 << SPI_CR1_BIDIMODE);

		//RXONLY IS SET TO ENABLE RECIEVE ONLY IN MASTER
		temp |= (1 << SPI_CR1_RXONLY);

	}

	//CONFIGURE THE CLOCK SPEED OF THE SPIx
	temp |= (pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR);

	//CONFIGURE THE DATA FRAME FORMAT
	temp |= (pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF);

	//CONGFIGURE THE CLOCK POLARITY
	temp |= (pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL);

	//CONFIGURE THE CLOCK PHASE
	temp |= (pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA);

	//CONFIG S/W SLAVE MANAGEMENT
	temp |= (pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM);


	//SAVE THE CONFIGURATION VARIABLE IN THE CR REGISTER

	pSPIHandle->pSPIx->CR1 |= temp;

}

/********************************************************************************
 * 							DOCUMENTATION
 * @fn		:
 *
 * @brief	:
 *
 * @param	:
 * @param	:
 * @param	:
 *
 * @return  :
 *
 * @Note	:
 */
void SPI_DeInit(SPI_RegDef_t *pSPIx)										//DEINITIALIZES SPI PORT
{
	//TODO
}

/********************************************************************************
 * 							DOCUMENTATION
 * @fn		:
 *
 * @brief	:
 *
 * @param	:
 * @param	:
 * @param	:
 *
 * @return  :
 *
 * @Note	:	THIS IS BLOCKING CALL / POLLING BASED
 */

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTXBuffer, uint32_t Len)
{

	//SEND INFO ABOUT THE LENGTH
	//pSPIx->DR = Len;

	//CHECK FOR THE LENGTH VARIABLE -- EXIT IF 0
	while(Len != 0)
	{
		//WAIT UNTIL TX BUFFER IS EMPTY -- CHECK TXE
		while( SPI_GetFagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET );

		//ONCE TX BUFFER IS EMPTY -- CHECK DFF
		if( !(pSPIx->CR1 & (0x1 << SPI_CR1_DFF)) )
		{

			//ONCE DFF IS CHECKED -- LOAD DR WITH 1 BYTE DATA
			pSPIx->DR = *pTXBuffer;

			//INCREMENT THE BUFFER ADDRESS
			pTXBuffer++;

			//DECREMENT THE LENGTH
			Len--;

		}else if(pSPIx->CR1 & (0x1 << SPI_CR1_DFF))
		{

			//ONCE DFF IS CHECKED -- LOAD DR WITH 2 BYTE DATA
			pSPIx->DR = *((uint16_t *)pTXBuffer);

			//INCREMENT THE BUFFER ADDRESS
			(uint16_t *)pTXBuffer++;

			//DECREMENT THE LENGTH
			Len -= 2;

		}

	}

}

/********************************************************************************
 * 							DOCUMENTATION
 * @fn		:
 *
 * @brief	:
 *
 * @param	:
 * @param	:
 * @param	:
 *
 * @return  :
 *
 * @Note	:
 */
void SPI_RecieveData(SPI_RegDef_t *pSPIx, uint8_t *pRXBuffer, uint32_t Len)
{

	//CHECK IF LENGTH  == 0 -- EXIT IF LENGTH = 0 i.e. DATA RECEIVED COMPLETELY
	while(Len != 0)
	{
		//WAIT UNTIL Rx BUFFER HAS NOT RECEIVED DATA
		while(SPI_GetFagStatus(pSPIx, SPI_RXNE_FLAG) != FLAG_SET);

		//ONCE DATA IS RECEIVED IN THE BUFFER THE RXNE FLAG IS SET AND NOW READ THE DATA FROM DR
		if( !(pSPIx->CR1 & (0x1 << SPI_CR1_DFF)) )
		{

			//ONCE DFF IS CHECKED  -- LOAD WITH 1 BYTE OF DATA
			*pRXBuffer = pSPIx->DR;

			//INCREMENT THE BUFFER ADDRESS
			pRXBuffer++;

			//DECREMENT THE LENGTH
			Len--;
		}
		else
		{
			//ONCE DFF IS CHECKED  -- LOAD WITH 2 BYTE OF DATA
			*((uint16_t *)pRXBuffer) = pSPIx->DR;

			//INCREMENT THE BUFFER ADDRESS
			(uint16_t *)pRXBuffer++;

			//DECREMENT THE LENGTH
			Len -= 2;
		}

	}
}

/********************************************************************************
 * 							DOCUMENTATION
 * @fn		:
 *
 * @brief	:
 *
 * @param	:
 * @param	:
 * @param	:
 *
 * @return  :
 *
 * @Note	:
 */

void SPI_IRQ_ITConfig(uint8_t IRQNumber, uint8_t En_Di)					//CONFIGURES IRQ
{

}

/********************************************************************************
 * 							DOCUMENTATION
 * @fn		:
 *
 * @brief	:
 *
 * @param	:
 * @param	:
 * @param	:
 *
 * @return  :
 *
 * @Note	:
 */
void SPI_IRQHandling(SPI_Handle_t *pHandle)								//HANDLER CODE FOR ISR
{
	uint8_t temp1 , temp2;

	//CHECK FOR SR_TXE
	temp1 = pHandle->pSPIx->SR  & (0x1 << SPI_SR_TXE);
	temp2 = pHandle->pSPIx->CR2 & (0x1 << SPI_CR2_TXEIE);

	if(temp1 && temp2)
	{
		SPI_TXE_IT_Handle(pHandle);											//HANDLER CODE FOR Tx BUFFER EMPTY
	}


	//CHECK FOR SR_RXNE
	temp1 = pHandle->pSPIx->SR  & (0x1 << SPI_SR_RXNE);
	temp2 = pHandle->pSPIx->CR2 & (0x1 << SPI_CR2_RXNEIE);

	if(temp1 && temp2)
	{
		SPI_RXNE_IT_Handle(pHandle);										//HANDLER CODE FOR Rx BUFFER NOT EMPTY
	}

	//CHECK FOR SR_OVR
	temp1 = pHandle->pSPIx->SR  & (0x1 << SPI_SR_OVR);
	temp2 = pHandle->pSPIx->CR2 & (0x1 << SPI_CR2_ERRIE);

	if(temp1 && temp2)
	{
		SPI_OVR_IT_Handle(pHandle);											//HANDLER CODE FOR OVER RUN ERR
	}


}


/********************************************************************************
 * 							DOCUMENTATION
 * @fn		:
 *
 * @brief	:
 *
 * @param	:
 * @param	:
 * @param	:
 *
 * @return  :
 *
 * @Note	:
 */

uint8_t SPI_SendData_IT(SPI_Handle_t *pSPIHandle, uint8_t *pTXBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->TxState;

	if(state != SPI_BUSY_Tx)
	{
		//1) SAVE THE Tx BUFFER ADDR && LENGTH INFO IN SOME GLOABL VARIABLE
		pSPIHandle->pTxBuffer = pTXBuffer;
		pSPIHandle->TxLen = Len;

		//2) MARK THE SPI STATE AS BUSY IN TRANSMISSION -- SO NO OTHER CODE CAN TAKE OVER THE SAME SPI PERIPHERAL UNTIL THE TRANSMISSION IS OVER
		pSPIHandle->TxState = SPI_BUSY_Tx;

		//3) ENABLE THE TXEIE CONTROL BIT TO GET INTERRUPT WHENEVER TXE FLAG IS SET IN SR
		pSPIHandle->pSPIx->CR2 |= (0x1 << SPI_CR2_TXEIE);

	}

	return state;

}


uint8_t SPI_RecieveData_IT(SPI_Handle_t *pSPIHandle, uint8_t *pRXBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->RxState;

	if(state != SPI_BUSY_Rx)
	{
		//1) SAVE THE Rx BUFFER ADDR && LENGTH INFO IN SOME GLOABL VARIABLE
		pSPIHandle->pRxBuffer = pRXBuffer;
		pSPIHandle->RxLen = Len;

		//2) MARK THE SPI STATE AS BUSY IN TRANSMISSION -- SO NO OTHER CODE CAN TAKE OVER THE SAME SPI PERIPHERAL UNTIL THE TRANSMISSION IS OVER
		pSPIHandle->RxState = SPI_BUSY_Rx;

		//3) ENABLE THE RXNEIE CONTROL BIT TO GET INTERRUPT WHENEVER TXE FLAG IS SET IN SR
		pSPIHandle->pSPIx->CR2 |= (0x1 << SPI_CR2_RXNEIE);

	}

	return state;

}

void SPI_IRQConfig(uint8_t IRQNumber,uint32_t IRQPriority)					// SPI PRIORITY HANDLER
{

}

static void SPI_RXNE_IT_Handle(SPI_Handle_t *pSPIHandle)
{

	//ONCE RX BUFFER IS FULL -- READ DFF
	if( !(pSPIHandle->pSPIx->CR1 & (0x1 << SPI_CR1_DFF)))
	{

		//ONCE DFF IS CHECKED -- READ THE DR WITH 1 BYTE DATA
		*(uint8_t *)pSPIHandle->pRxBuffer =  pSPIHandle->pSPIx->DR;

		//INCREMENT THE BUFFER ADDR
		(uint8_t *)(pSPIHandle->pRxBuffer)++;

		//DECREMENT THE RX LENGTH
		(pSPIHandle->RxLen)--;
	}else if( pSPIHandle->pSPIx->CR1 & (0x1 << SPI_CR1_DFF))
	{

		//ONCE DFF IS CHECKED  -- READ THE DR WITH 2 BYTES OF DATA
		*(uint16_t *)pSPIHandle->pRxBuffer = pSPIHandle->pSPIx->DR;

		//INCREMENT THE BUFFER ADDR
		(uint16_t *)(pSPIHandle->pRxBuffer)++;

		//DECREMENT THE RX LENGTH BY 2
		pSPIHandle->RxLen -= 2;
	}

	if( !pSPIHandle->TxLen )
	{
		//Rx LENGTH IS 0 -- CLOSE THE SPI TRANSMISSION && INFORM THE APPLICATION THAT Rx HAS BEEN COMPLETED

		//THIS PREVENTS INTERRUPTS FROM SETTING UP THE Rx FLAG
		SPI_AbortRx(pSPIHandle);

		SPI_AppEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);

	}
}

static void SPI_TXE_IT_Handle(SPI_Handle_t *pSPIHandle)
{

	//ONCE TX BUFFER IS EMPTY -- CHECK DFF
	if( !(pSPIHandle->pSPIx->CR1 & (0x1 << SPI_CR1_DFF)) )
	{

		//ONCE DFF IS CHECKED -- LOAD DR WITH 1 BYTE DATA
		pSPIHandle->pSPIx->DR = *((uint8_t *)pSPIHandle->pTxBuffer);

		//INCREMENT THE BUFFER ADDRESS
		((uint8_t *)pSPIHandle->pTxBuffer++);

		//DECREMENT THE LENGTH
		(pSPIHandle->TxLen)--;

	}else if(pSPIHandle->pSPIx->CR1 & (0x1 << SPI_CR1_DFF))
	{

		//ONCE DFF IS CHECKED -- LOAD DR WITH 2 BYTE DATA
		pSPIHandle->pSPIx->DR = *((uint16_t *)pSPIHandle->pTxBuffer);

		//INCREMENT THE BUFFER ADDRESS
		((uint16_t *)pSPIHandle->pTxBuffer++);

		//DECREMENT THE LENGTH
		(pSPIHandle->TxLen) -= 2;

	}

	if( !pSPIHandle->TxLen )
	{
		//Tx LENGTH IS 0 -- CLOSE THE SPI TRANSMISSION && INFORM THE APPLICATION THAT Tx HAS BEEN COMPLETED

		//THIS PREVENTS INTERRUPTS FROM SETTING UP THE Tx FLAG
		SPI_AbortTx(pSPIHandle);

		SPI_AppEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);

	}

}

static void SPI_OVR_IT_Handle(SPI_Handle_t *pSPIHandle)
{

	//CLEAR THE OVR FLAG
	if(pSPIHandle->TxState == SPI_BUSY_Tx)
	{
		uint8_t temp;
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;

		(void)temp;
	}

	//INFORM THE APPLICATION
	SPI_AppEventCallback(pSPIHandle, SPI_EVENT_OVR_ERR);
}

void SPI_AbortTx(SPI_Handle_t *pSPIHandle)
{

	pSPIHandle->pSPIx->CR2 &= ~( 0x1 << SPI_CR2_TXEIE );
	pSPIHandle->TxLen = 0;
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxState = SPI_READY;

}

void SPI_AbortRx(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~( 0x1 << SPI_CR2_RXNEIE );
	pSPIHandle->RxLen = 0;
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxState = SPI_READY;

}

void SPI_ClearOVR(SPI_RegDef_t *pSPIHandle)
{

	//READ THE DR FOLLOWED BY SR
	uint8_t temp;

	temp = pSPIHandle->DR;
	temp = pSPIHandle->SR;

	(void)temp;
}

__weak void SPI_AppEventCallback(SPI_Handle_t *pSPIHandle, uint8_t EventCall)
{
	//THIS IS A WEAK IMPLEMENTATION -- THE USER MAY OVERRIDE THIS


}








