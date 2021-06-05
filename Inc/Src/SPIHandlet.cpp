/*
 * SPIHandlet.cpp
 *
 *  Created on: 01-Jun-2021
 *      Author: root
 */

#include <SPIHandlet.h>


SPI_Config_t::SPI_Config_t(){
SPI_DeviceMode=0 ;	
SPI_BusConfig=0;	
SPI_SclkSpeed=0;	
SPI_DFF=0;		
SPI_CPOL=0;		
SPI_CPHA=0;		
SPI_SSM=0;		
}

SPI_Config_t:: ~SPI_Config_t(){}


SPI_Handle_t::SPI_Handle_t() {
	// TODO Auto-generated constructor stub

}

SPI_Handle_t::~SPI_Handle_t() {
	// TODO Auto-generated destructor stub
}

static void SPI_RXNE_IT_Handle(SPI_Handle_t *pSPIHandle);
static void SPI_TXE_IT_Handle(SPI_Handle_t *pSPIHandle);
static void SPI_OVR_IT_Handle(SPI_Handle_t *pSPIHandle);

/********************************************************************************
 * 							DOCUMENTATION
 * @fn		: uint8_t SPI_GetFagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
 *
 * @brief	: returns status of the respective register flag
 *
 * @param	: SPI_RegDef_t *pSPIx
 * @param	: uint32_t FlagName
 *
 * @return  : uint8_t
 *
 * @Note	:
 */
uint8_t SPI_GetFagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
	if(pSPIx->SR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/********************************************************************************
 * 							DOCUMENTATION
 *
 * @fn		: void SPI_Enable(SPI_RegDef_t *SPI_ENDI, uint8_t EN_DI)
 *
 * @brief	: enables/disables the respective SPI peripheral
 *
 * @param	: SPI_RegDef_t *SPI_ENDI
 * @param	: uint8_t EN_DI
 *
 * @return  : void
 *
 * @Note	: NULL
 */
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

/********************************************************************************
 * 							DOCUMENTATION
 *
 * @fn		: void SSI_Config(SPI_RegDef_t *SPI_ENDI, uint8_t EN_DI)
 *
 * @brief	: This enables/disables the Software Slave Management
 *
 * @param	: SPI_RegDef_t *SPI_ENDI
 * @param	: uint8_t EN_DI
 *
 * @return  : void
 *
 * @Note	: This feature allows automatic management of the CS pin
 */
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

/********************************************************************************
 * 							DOCUMENTATION
 *
 * @fn		: void SSOE_Config(SPI_RegDef_t *SPI_ENDI, uint8_t EN_DI)
 *
 * @brief	: Enables/Disables Software Slave Output Enable
 *
 * @param	: SPI_RegDef_t *SPI_ENDI
 * @param	: uint8_t EN_DI
 *
 * @return  : void
 *
 * @Note	: This needs to be enabled to use SSI feature
 */
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
 *
 * @fn		: void SPI_PeriClkCntrl(SPI_RegDef_t *pSPIx, uint8_t En_Di)
 *
 * @brief	: Enables/Disables the SPI peripheral clock
 *
 * @param	: SPI_RegDef_t *pSPIx
 * @param	: uint8_t En_Di
 *
 * @return  : void
 *
 * @Note	: NULL
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
 *
 * @fn		: void SPI_Init(SPI_Handle_t *pSPIHandle)
 *
 * @brief	: Initializes the SPI peripheral according according to SPI_Handle_t
 *
 * @param	: SPI_Handle_t *pSPIHandle
 *
 * @return  : void
 *
 * @Note	: NULL
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
 *
 * @fn		: void SPI_DeInit(SPI_RegDef_t *pSPIx)
 *
 * @brief	: Deinits the given SPI peripheral
 *
 * @param	: SPI_RegDef_t *pSPIx
 *
 * @return  : void
 *
 * @Note	: NULL
 */
void SPI_DeInit(SPI_RegDef_t *pSPIx)										//DEINITIALIZES SPI PORT
{

	if(pSPIx == SPI1)
	{
		SPI1_REG_RESET();
	}else if(pSPIx == SPI2)
	{
		SPI2_REG_RESET();
	}else if(pSPIx == SPI2)
	{
		SPI2_REG_RESET();
	}else if(pSPIx == SPI3)
	{
		SPI3_REG_RESET();
	}else if(pSPIx == SPI4)
	{
		SPI4_REG_RESET();
	}

}

/********************************************************************************
 * 							DOCUMENTATION
 *
 * @fn		: void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTXBuffer, uint32_t Len)
 *
 * @brief	: Transmits the data
 *
 * @param	: SPI_RegDef_t *pSPIx
 * @param	: uint8_t *pTXBuffer
 * @param	: uint32_t Len
 *
 * @return  : void
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
 *
 * @fn		: void SPI_RecieveData(SPI_RegDef_t *pSPIx, uint8_t *pRXBuffer, uint32_t Len)
 *
 * @brief	: Receives data through SPI peripheral of the given data length
 *
 * @param	: SPI_RegDef_t *pSPIx
 * @param	: uint8_t *pRXBuffer
 * @param	: uint32_t Len
 *
 * @return  : void
 *
 * @Note	: NULL
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
 *
 * @fn		: void SPI_IRQ_ITConfig(uint8_t IRQNumber, uint8_t En_Di)					//CONFIGURES IRQ
 *
 * @brief	: Enables/Disables IRQNumber of the SPI peripheral
 *
 * @param	: uint8_t IRQNumber
 * @param	: uint8_t En_Di
 *
 * @return  : void
 *
 * @Note	: NULL
 */

void SPI_IRQ_ITConfig(uint8_t IRQNumber, uint8_t En_Di)					//CONFIGURES IRQ
{
	if(En_Di == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			//SET ISER0
			*NVIC_ISER0 |= (0x1 << IRQNumber);
		}
		else if(IRQNumber <= 63 && IRQNumber >31)
		{
			//SET ISER1
			*NVIC_ISER1 |= (0x1 << (IRQNumber % 32));
		}
		else if(IRQNumber <= 95 && IRQNumber >63)
		{
			//SET ISER2
			*NVIC_ISER2 |= (0x1 << (IRQNumber % 64));
		}
	}

	if(En_Di == DISABLE)
	{
		if(IRQNumber <= 31)
		{
			//SET ICER0
			*NVIC_ICER0 |=  (0x1 << IRQNumber);
		}
		else if(IRQNumber <= 63 && IRQNumber >31)
		{
			//SET ICER1
			*NVIC_ICER1 |=  (0x1 << (IRQNumber % 32));
		}
		else if(IRQNumber <= 95 && IRQNumber >63)
		{
			//SET ICER2
			*NVIC_ICER2 |=  (0x1 << (IRQNumber % 64));
		}
	}
}

/********************************************************************************
 * 							DOCUMENTATION
 *
 * @fn		: void SPI_IRQHandling(SPI_Handle_t *pHandle)
 *
 * @brief	: Handler for SPI peripheral
 *
 * @param	: SPI_Handle_t *pHandle
 *
 * @return  : void
 *
 * @Note	: NULL
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
 *
 * @fn		: uint8_t SPI_SendData_IT(SPI_Handle_t *pSPIHandle, uint8_t *pTXBuffer, uint32_t Len)
 *
 * @brief	: Interrupt based function to transmit data through SPI, len amount of data to be sent
 *
 * @param	: SPI_Handle_t *pSPIHandle
 * @param	: uint8_t *pTXBuffer
 * @param	: uint32_t Len
 *
 * @return  : uint8_t
 *
 * @Note	: Interrupt based call
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


/********************************************************************************
 * 							DOCUMENTATION
 *
 * @fn		: uint8_t SPI_RecieveData_IT(SPI_Handle_t *pSPIHandle, uint8_t *pRXBuffer, uint32_t Len)
 *
 * @brief	: Interrupt based function to receive data through SPI, len amount of data to be received
 *
 * @param	: SPI_Handle_t *pSPIHandle
 * @param	: uint8_t *pRXBuffer
 * @param	: uint32_t Len
 *
 * @return  : uint8_t
 *
 * @Note	: returns state
 */
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

/********************************************************************************
 * 							DOCUMENTATION
 *
 * @fn		: void SPI_IRQConfig(uint8_t IRQNumber,uint32_t IRQPriority)					// SPI PRIORITY HANDLER
 *
 * @brief	: Configures the IRQ Priority of the given IRQ number
 *
 * @param	: uint8_t IRQNumber
 * @param	: uint32_t IRQPriority
 *
 * @return  : void
 *
 * @Note	: NULL
 */
void SPI_IRQConfig(uint8_t IRQNumber,uint32_t IRQPriority)					// SPI PRIORITY HANDLER
{
	uint8_t iprx 		= IRQNumber / 4;
	uint8_t ipr_section = IRQNumber % 4;

	uint8_t shift_amnt  = ((8 * ipr_section) + (8 - NO_PRIORITY_BITS));
	*( NVIC_IPR_BA + (iprx) ) |= (IRQPriority << shift_amnt);
}

/********************************************************************************
 * 							DOCUMENTATION
 *
 * @fn		: static void SPI_RXNE_IT_Handle(SPI_Handle_t *pSPIHandle)
 *
 * @brief	: Handle function for SPI Receive - Interrupt based
 *
 * @param	: SPI_Handle_t *pSPIHandle
 *
 * @return  : void
 *
 * @Note	: private function
 */
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

/********************************************************************************
 * 							DOCUMENTATION
 *
 * @fn		: static void SPI_TXE_IT_Handle(SPI_Handle_t *pSPIHandle)
 *
 * @brief	: Handle function for SPI Transmit - Interrupt based
 *
 * @param	: SPI_Handle_t *pSPIHandle
 *
 * @return  : void
 *
 * @Note	: private function
 */
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

/********************************************************************************
 * 							DOCUMENTATION
 *
 * @fn		: static void SPI_OVR_IT_Handle(SPI_Handle_t *pSPIHandle)
 *
 * @brief	: Handle function for Overrun ERROR
 *
 * @param	: SPI_Handle_t *pSPIHandle
 *
 * @return  : void
 *
 * @Note	: Private function
 */
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

/********************************************************************************
 * 							DOCUMENTATION
 *
 * @fn		: void SPI_AbortTx(SPI_Handle_t *pSPIHandle)
 *
 * @brief	: Aborts Transmission of the given SPI peripheral
 *
 * @param	: SPI_Handle_t *pSPIHandle
 *
 * @return  : void
 *
 * @Note	: NULL
 */
void SPI_AbortTx(SPI_Handle_t *pSPIHandle)
{

	pSPIHandle->pSPIx->CR2 &= ~( 0x1 << SPI_CR2_TXEIE );
	pSPIHandle->TxLen = 0;
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxState = SPI_READY;

}

/********************************************************************************
 * 							DOCUMENTATION
 *
 * @fn		: void SPI_AbortRx(SPI_Handle_t *pSPIHandle)
 *
 * @brief	: Aborts Reception of the given SPI peripheral
 *
 * @param	: SPI_Handle_t *pSPIHandle
 *
 * @return  : SPI_Handle_t *pSPIHandle
 *
 * @Note	: NULL
 */
void SPI_AbortRx(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~( 0x1 << SPI_CR2_RXNEIE );
	pSPIHandle->RxLen = 0;
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxState = SPI_READY;

}

/********************************************************************************
 * 							DOCUMENTATION
 *
 * @fn		: void SPI_ClearOVR(SPI_RegDef_t *pSPIHandle)
 *
 * @brief	: Clears the Overrun ERROR
 *
 * @param	: SPI_RegDef_t *pSPIHandle
 *
 * @return  : void
 *
 * @Note	: NULL
 */
void SPI_ClearOVR(SPI_RegDef_t *pSPIHandle)
{

	//READ THE DR FOLLOWED BY SR
	uint8_t temp;

	temp = pSPIHandle->DR;
	temp = pSPIHandle->SR;

	(void)temp;
}

/********************************************************************************
 * 							DOCUMENTATION
 *
 * @fn		: __weak void SPI_AppEventCallback(SPI_Handle_t *pSPIHandle, uint8_t EventCall)
 *
 * @brief	: THis is a weak implementation. Don't remove this;
 * 			  Copy the function to the User file and remove the __weak attribute
 *
 * @param	: SPI_Handle_t *pSPIHandle
 * @param	: uint8_t EventCall
 *
 * @return  : void
 *
 * @Note	: NULL
 */
__weak void SPI_AppEventCallback(SPI_Handle_t *pSPIHandle, uint8_t EventCall)
{
	//THIS IS A WEAK IMPLEMENTATION -- THE USER MAY OVERRIDE THIS
}