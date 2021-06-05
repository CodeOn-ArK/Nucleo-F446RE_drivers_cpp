/*
 * I2CHandlet.cpp
 *
 *  Created on: 01-Jun-2021
 *      Author: root
 */

#include <I2CHandlet.h>

I2C_Config_t:: I2C_Config_t(){
    I2C_SCLSpeed = 0	;	
	I2C_DeviceAddress =0 ;	
	I2C_ACKControl=0	;	
	I2C_FMDutyCycle	=0;
}

I2C_Config_t:: ~I2C_Config_t(){}

I2C_Handle_t::I2C_Handle_t() {
	// TODO Auto-generated constructor stub
	

}

I2C_Handle_t::~I2C_Handle_t() {
	// TODO Auto-generated destructor stub
}

uint32_t AHB1P[]  = {2, 4, 8, 16, 64, 128, 256, 512};
uint32_t APB1p[]  =  {2, 4, 8, 16};

/*
 * exported variable
 */
extern uint8_t cmnd_code;

/*
 * private Macros
 */
#define READ 	1
#define WRITE	0

/*
 * Private function starting here
 */
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle);
static void I2C_GenerateStartCondition(I2C_RegDef_t *);
static void I2C_ExecuteAddrPhase(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr, uint8_t);

/*
 * Global function starting here
 */
void I2C_CallBack();

/******************************************************************************************
 * 							Code Implementation starts here
 ******************************************************************************************/

/*********************************************************************
 * @fn      		  - static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle )
 *
 * @brief             - Clears the ADDR FLAG of the SR2 reg of the corresponding I2C peripheral
 *
 * @param[in]         -	I2C_Handle_t *pI2CHandle
 *
 * @return            - void
 *
 * @Note              - private function
 */
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle )
{
	uint32_t dummy_read;
	//check for device mode
	if(pI2CHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_MSL))
	{
		//device is in master mode
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_Rx)
		{
			if(pI2CHandle->RxSize  == 1)
			{
				//first disable the ack
				pI2CHandle->pI2Cx->CR1 &= ~(0x1 << I2C_CR1_ACK);//I2C_ManageAcking(pI2CHandle->pI2Cx,DISABLE);

				//clear the ADDR flag ( read SR1 , read SR2)
				dummy_read = pI2CHandle->pI2Cx->SR1;
				dummy_read = pI2CHandle->pI2Cx->SR2;
				(void)dummy_read;
			}

		}
		else
		{
			//clear the ADDR flag ( read SR1 , read SR2)
			dummy_read = pI2CHandle->pI2Cx->SR1;
			dummy_read = pI2CHandle->pI2Cx->SR2;
			(void)dummy_read;

		}

	}
	else
	{
		//device is in slave mode
		//clear the ADDR flag ( read SR1 , read SR2)
		dummy_read = pI2CHandle->pI2Cx->SR1;
		dummy_read = pI2CHandle->pI2Cx->SR2;
		(void)dummy_read;
	}


}

/*********************************************************************
 * @fn      		  - static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
 *
 * @brief             - Generates START condition at the corresponding I2C peripheral pI2Cx
 *
 * @param[in]         - I2C_RegDef_t *pI2Cx
 *
 * @return            - void
 *
 * @Note              - private function
 */
static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (0x1 << I2C_CR1_START);
}

/*********************************************************************
 * @fn      		  - static void I2C_ExecuteAddrPhase(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr, uint8_t mode)
 *
 * @brief             - Sends out the address on the data line(SDA) along with the
 * 						given mode(R/W') at the corresponding peripheral
 *
 * @param[in]         - I2C_RegDef_t *pI2Cx
 * @param[in]         - uint8_t SlaveAddr
 * @param[in]         - uint8_t mode
 *
 * @return            - void
 *
 * @Note              - private function
 */
static void I2C_ExecuteAddrPhase(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr, uint8_t mode)
{
	if(mode == WRITE)
	{
		pI2Cx->DR  =  ( (SlaveAddr << 1) | 0x0 );								//Shifting the slave address left by in byte bringing in a 0 from right which is the new LSB
																								//0 in LSB signifies master is going to write the data
	}
	else
	{
		pI2Cx->DR = ( (SlaveAddr << 1) |  0x1 );
	}
}

/*********************************************************************
 * @fn      		  - void I2C_Enable(I2C_RegDef_t *pI2Cx , uint8_t EN_DI)
 *
 * @brief             - Enables the I2C peripheral pI2Cx
 *
 * @param[in]         - I2C_RegDef_t *pI2Cx
 * @param[in]         - uint8_t EN_DI
 *
 * @return            - void
 *
 * @Note              - NULL
 */
void I2C_Enable(I2C_RegDef_t *pI2Cx , uint8_t EN_DI)
{
	if(EN_DI == ENABLE)
		pI2Cx->CR1 |= ( 0x1 << I2C_CR1_PE );

	else pI2Cx->CR1 &= ~(0x1 << I2C_CR1_PE);

}


/*********************************************************************
 * @fn      		  - uint8_t I2C_GetFagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName)
 *
 * @brief             - Returns the flag status of the corresponding flag register in SR1 of the
 * 						corresponding I2C peripheral
 *
 * @param[in]         - I2C_RegDef_t *pI2Cx
 * @param[in]         - uint32_t FlagName
 *
 * @return            - uint8_t
 *
 * @Note              - NULL
 */
uint8_t I2C_GetFagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName)
{
	if( pI2Cx->SR1 & (0x1 << FlagName) )
		return FLAG_SET;

	return FLAG_RESET;
}

/*********************************************************************
 * @fn      		  - uint32_t RCC_GetClkVal(void)
 *
 * @brief             - Returns the clock value of APB1 bus
 *
 * @param[in]         - void
 *
 * @return            - uint32_t
 *
 * @Note              - NULL
 */
uint32_t RCC_GetClkVal(void)
{
	uint32_t PCLK1, Clk_SCL, temp;
	uint8_t System_clk_status, AHB1_Prescalar, APB1_Prescalar;

	System_clk_status = ((RCC->CFGR >> 2 ) & (0x3));

	if(System_clk_status == 0)
	{
		//HSI
		Clk_SCL = 16000000;

	}else if(System_clk_status == 1)
	{
		//HSE
	}else if(System_clk_status == 2)
	{
		//PLL
	}else if(System_clk_status == 3)
	{
		//PLL_R
	}

	temp = ((RCC->CFGR >> 4) & (0x4));

	if(temp < 8)
	{
		AHB1_Prescalar = 1;
	}else
	{
		AHB1_Prescalar = (AHB1P[temp] - 8);
	}

	temp = ((RCC->CFGR >> 10 ) & (0x3));

	if(temp < 4)
	{
		APB1_Prescalar = 1;

	}else
	{
		APB1_Prescalar = (APB1p[temp] - 4);
	}

	PCLK1 = (Clk_SCL/ AHB1_Prescalar)/APB1_Prescalar;

	return PCLK1;
}


/*********************************************************************
 * @fn      		  - void I2C_PeriClkCntrl(I2C_RegDef_t *pI2Cx, uint8_t En_Di)
 *
 * @brief             - Enables/Disables the corresponding I2C peripheral pI2Cx
 *
 * @param[in]         - I2C_RegDef_t *pI2Cx
 * @param[in]         - uint8_t En_Di
 *
 * @return            - void
 *
 * @Note              - NULL
 */
void I2C_PeriClkCntrl(I2C_RegDef_t *pI2Cx, uint8_t En_Di)
{
	if(En_Di  ==	 ENABLE)
	{
		if(pI2Cx == I2C1)
			I2C1_CLK_EN();

		else if(pI2Cx == I2C2)
			I2C2_CLK_EN();

		else if(pI2Cx == I2C3)
			I2C2_CLK_EN();

	}
	else
	{
		if(pI2Cx == I2C1)
			I2C1_CLK_DI();

		else if(pI2Cx == I2C2)
			I2C2_CLK_DI();

		else if(pI2Cx == I2C3)
			I2C3_CLK_DI();

	}

}

/*********************************************************************
 * @fn      		  - void I2C_Init(I2C_Handle_t *pI2CHandle)
 *
 * @brief             - Initializes the corresponding I2C port ,in pI2CHandle, according to the structure pI2CHandle
 *
 * @param[in]         - I2C_Handle_t *pI2CHandle
 *
 * @return            - void
 *
 * @Note              - NULL
 */
void I2C_Init(I2C_Handle_t *pI2CHandle)				//INITIALIZES I2C PORT
{
	uint32_t tempreg = 0;

	//Enable the I2Cx peripheral clock
	I2C_PeriClkCntrl(pI2CHandle->pI2Cx , ENABLE);

	//ack control bit
	tempreg = 0;
	tempreg = (pI2CHandle->I2C_Config.I2C_ACKControl << 10);
	pI2CHandle->pI2Cx->CR1 |= tempreg;

	//prog the device own address
	tempreg = 0;
	tempreg = pI2CHandle->I2C_Config.I2C_DeviceAddress << 1;
	tempreg |= (0x1 << 14);
	pI2CHandle->pI2Cx->OAR1 |= tempreg;

	//config the FREQ field of CR2
	tempreg = 0;
	tempreg = RCC_GetClkVal()/1000000U;
	pI2CHandle->pI2Cx->CR2 |= (tempreg & ( 0x3F));
/*
 * Configuring I2C SCL :
 *
 * For SM mode;
 * 1) Config the mode in CCR reg
 * 2) prog FREQ field of CR2 with the value of PCLK1 (The APB to which the I2C is connected)
 * 3) Calc && prog the CCR value in CCr field of CCR reg
 *
 * {T refers to time period}
 *
 * T(high(SCL)) = CCR * T(PCLK1)
 * T(low(SCL)  = CCR *T(PLCK1)
 *
 *For FM mode;
 *1) Config the mode in CCR reg
 *2)Select the duty cycle of Fast mode SCL in CCR reg(14th bit)
 *3) Prog FREQ field of CR2 with the value of PCLK1
 *4) Calc and prog CCR value in CCr in CCR field of CCR reg
 *
 *if DUTY=0
 * T(HIGH)   =   CCR  *  T(PCLK1)
 * T(LOW)		=  2 * CCR  * T(PCLK1)
 *
 * if DUTY=1
 * T(HIGH)	=  9 * CCR * T(PCLK1)
 * T(HIGH)	=	16 * CCR  *  T(PCLK1)
 *
 *
 */

	//CCR Calculations
	uint16_t ccr_value = 0;
	tempreg = 0;

	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		ccr_value =  (RCC_GetClkVal() / (2 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		tempreg = (ccr_value & (0xFFF));

		pI2CHandle->pI2Cx->CCR |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14 );
		pI2CHandle->pI2Cx->CCR &= ~(0x1 << 15);
		pI2CHandle->pI2Cx->CCR |= (tempreg);

	}else
	{
		if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2)
		{
			ccr_value = (RCC_GetClkVal())/(3 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
			tempreg = (ccr_value & (0xFFF));

			pI2CHandle->pI2Cx->CCR |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14 );
			pI2CHandle->pI2Cx->CCR |= (0x1 << 15);
			pI2CHandle->pI2Cx->CCR |= (tempreg);

		}else
		{
			ccr_value = (RCC_GetClkVal())/(25 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
			tempreg = (ccr_value & (0xFFF));

			pI2CHandle->pI2Cx->CCR |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14 );
			pI2CHandle->pI2Cx->CCR |= (0x1 << 15);
			pI2CHandle->pI2Cx->CCR |= (tempreg);

		}
	}

	uint32_t temp = 0;

	//TRISE Configuration
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		temp = ((( RCC_GetClkVal()*1 )/1000000U ) + 1);
	}else
	{
		temp = ((( RCC_GetClkVal()*300 )/1000000000U ) + 1);
	}

	pI2CHandle->pI2Cx->TRISE |= ((temp) & (0x3F));
	(void)temp;

	pI2CHandle->pI2Cx->CR1 |= (0x1 << I2C_CR1_PE);
}


/*********************************************************************
 * @fn      		  - void I2C_DeInit(I2C_RegDef_t *pI2Cx)
 *
 * @brief             - DeInitializes the I2C port
 *
 * @param[in]         - I2C_RegDef_t *pI2Cx
 *
 * @return            - void
 *
 * @Note              - NULL
 */
void I2C_DeInit(I2C_RegDef_t *pI2Cx)										//DEINITIALIZES I2C PORT
{
	if(pI2Cx == I2C1)
	{
		I2C1_REG_RESET();
	}
	else if(pI2Cx == I2C2)
	{
		I2C2_REG_RESET();
	}
	else if(pI2Cx == I2C3)
	{
		I2C3_REG_RESET();
	}

}

/*********************************************************************
 * @fn      		  - void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTXBuffer, uint32_t Len, uint8_t SlaveAddr)
 *
 * @brief             - Sends data of the specified length through the specified I2C port to the specified slaveaddress
 *
 * @param[in]         - I2C_Handle_t *pI2CHandle
 * @param[in]         - uint8_t *pTXBuffer
 * @param[in]         - uint32_t Len
 * @param[in]         - uint8_t SlaveAddr *
 *
 * @return            - void
 *
 * @Note              - NULL
 */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTXBuffer, uint32_t Len, uint8_t SlaveAddr)
{

	//1. 	Generate the start condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//2.  Confirm that start Generation is completed by checking the SB flag in the SR1
	// Note: Until SB is cleared SCL will be stretched (pulled to LOW)
	while( !(I2C_GetFagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SR1_SB)) );						//Be in while loop until SBin SR is set

	//3.   Send the address of the slave with R/nW bit set to w(0) (total 8 bits)
	I2C_ExecuteAddrPhase(pI2CHandle->pI2Cx, SlaveAddr, WRITE);

	//4.   Confirm that address phase is completed by checking the ADDR flag in the SR1
	while( !(pI2CHandle->pI2Cx->SR1 & (0x1 << I2C_SR1_ADDR)) );				//wait until ADDR bit of SR1 is set indicating end of address transmission

	//5.	Clear the ADDR flag according to its s/w response
	//Note:	Until the ADDR is cleared SCL will be stretched(pulled to LOW)
	uint32_t temp;

	temp = pI2CHandle->pI2Cx->SR1;
	temp = pI2CHandle->pI2Cx->SR2;

	//6.	Send the data until Len becomes 0
	while(Len > 0)
	{
		while( !(I2C_GetFagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SR1_TxE)) );

		pI2CHandle->pI2Cx->DR  =  *(pTXBuffer);
		pTXBuffer++;
		Len--;
	}

	//7.	When Len becomes zero wait for TXE = 1 && BTF = 1 before generating the STOP condition
	//Note: 	TXE=1, BTF=1 , means that both SR && DR are empty && nxt transmission should begin
	//				when BTF=1 SCL will be stretched (pulled to LOW)
	while(  !( (pI2CHandle->pI2Cx->SR1 >> I2C_SR1_TxE  &  0x1) && (pI2CHandle->pI2Cx->SR1 >> I2C_SR1_BTF & 0x1)  )  );


	//8.	Generate STOP condition && master need not to wait for the completion of STOP condition.
	//Note:	generating STOP, automatically clears the BTF
	pI2CHandle->pI2Cx->CR1 |= (0x1 << I2C_CR1_STOP);

	(void)temp;
	I2C_AppEventCallback(pI2CHandle, I2C_EV_Tx_COMPLETE);
}

/*********************************************************************
 * @fn      		  - void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRXBuffer, uint32_t Len, uint8_t SlaveAddr)
 *
 * @brief             - Receives data of the specified length over the given I2C peripheral from the mentioned SlaveAddr
 *
 * @param[in]         - I2C_Handle_t *pI2CHandle
 * @param[in]         - uint8_t *pRXBuffer
 * @param[in]         - uint32_t Len
 * @param[in]         - uint8_t SlaveAddr
 *
 * @return            - void
 *
 * @Note              - NULL
 */
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRXBuffer, uint32_t Len, uint8_t SlaveAddr)
{
	uint32_t tempreg;

	//1. Generate the START condition
	pI2CHandle->pI2Cx->CR1 |= (0x1 << I2C_CR1_START);

	//2. COnfirm that start generation is completed by checking the SB flag in the SR1
	//Note: Unitl SB is cleared SCl will be stretched (Pulled Low)
	while( !(I2C_GetFagStatus(pI2CHandle->pI2Cx, I2C_SR1_SB)) );

	//3. Send the address of the slave with R/nW  bit set to R(1) (total 8 bits)
	I2C_ExecuteAddrPhase(pI2CHandle->pI2Cx, SlaveAddr, READ);

	//4. Wait until addr phase is completed by checking the ADDR flag in the SR1
	while( !(I2C_GetFagStatus(pI2CHandle->pI2Cx, I2C_SR1_ADDR)) );

	//Procedure to read only 1 byte of data
	if(Len == 1)
	{
		// Disable Acking
		pI2CHandle->pI2Cx->CR1 &= ~(0x1 << I2C_CR1_ACK);

		// Clear the ADDR flag
		tempreg = pI2CHandle->pI2Cx->SR1;
		tempreg = pI2CHandle->pI2Cx->SR2;

		// Wait until RXNE becomes 1
		while( !(I2C_GetFagStatus(pI2CHandle->pI2Cx, I2C_SR1_RxNE)) );

		//Generate STOP condition
		pI2CHandle->pI2Cx->CR1 |= (0x1 << I2C_CR1_STOP);

		// Read data in to buffer
		*pRXBuffer |= pI2CHandle->pI2Cx->DR;

		pI2CHandle->RxSize = *pRXBuffer;
	}

	//procedure to READ data from Slave when Len > 1
	if(Len > 1)
	{
		//Clear the ADDR flag
		tempreg = pI2CHandle->pI2Cx->SR2;

		//Read the data until Len becomes Zero
		for(uint32_t i = Len+1; i > 1; i--)
		{
			if(i == 2)
			{
				//Clear the ACK bit
				pI2CHandle->pI2Cx->CR1 &= ~(0x1 << I2C_CR1_ACK);

				//Generate STOP condtion
				pI2CHandle->pI2Cx->CR1 |= (0x1 << I2C_CR1_STOP);
			}

			//Wait until RXNE becomes 1
			while( !(I2C_GetFagStatus(pI2CHandle->pI2Cx, I2C_SR1_RxNE)) );

			//Read the data from data reg in to buffer
			*pRXBuffer = pI2CHandle->pI2Cx->DR;

			//Increment the buffer Addr
			pRXBuffer++;
		}
	}

	//Re-enable ACKing
	if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_EN)
	{
		pI2CHandle->pI2Cx->CR1 |= (0x1 << I2C_CR1_ACK);
	}

	(void)tempreg;

	I2C_AppEventCallback(pI2CHandle, I2C_EV_Rx_COMPLETE);
}


/*********************************************************************
 * @fn      		  -  uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t Sr)
 *
 * @brief             - Interrupt based Reception from the given SlaveAddr over the given I2C peripheral
 *
 * @param[in]         - I2C_Handle_t *pI2CHandle
 * @param[in]         - uint8_t *pRxBuffer
 * @param[in]         - uint32_t Len
 * @param[in]         - uint8_t SlaveAddr
 * @param[in]         - uint8_t Sr
 *
 * @return            - uint8_t (success status @I2C_application_state)
 *
 * @Note              - You can enable/disable repeated start using the Sr field
 */
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t Sr)
{

	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_BUSY_IN_Tx) && (busystate != I2C_BUSY_IN_Rx))
	{
		pI2CHandle->pRxBuffer = pRxBuffer;
		pI2CHandle->RxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_Rx;
		pI2CHandle->RxSize = Len; //Rxsize is used in the ISR code to manage the data reception
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (0x1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (0x1 << I2C_CR2_ITEVTEN);

		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (0x1 << I2C_CR2_ITERREN);

		//Implement code to Generate START Condition
		pI2CHandle->pI2Cx->CR1 |= (0x1 << I2C_CR1_START);
	}

	return busystate;
}

/*********************************************************************
 * @fn      		  - uint8_t  I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pTxBuffer, uint32_t Len,uint8_t SlaveAddr,uint8_t Sr)
 *
 * @brief             - Interrupt based Transmission to the given SlaveAddr over the given I2C peripheral
 *
 * @param[in]         - I2C_Handle_t *pI2CHandle
 * @param[in]         - uint8_t *pTxBuffer
 * @param[in]         - uint32_t Len
 * @param[in]         - uint8_t SlaveAddr
 * @param[in]         - uint8_t Sr
 *
 * @return            - uint8_t (success status @I2C_application_state)
 *
 * @Note              - You can enable/disable Repeated Start using the Sr field
 */
uint8_t  I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pTxBuffer, uint32_t Len,uint8_t SlaveAddr,uint8_t Sr)
{

	uint8_t busystate = (pI2CHandle->TxRxState);

	if( (busystate != I2C_BUSY_IN_Tx) && (busystate != I2C_BUSY_IN_Rx))
	{
		pI2CHandle->pTxBuffer = pTxBuffer;
		pI2CHandle->TxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_Tx;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (0x1 << I2C_CR2_ITEVTEN);

		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (0x1 << I2C_CR2_ITERREN);

		//Implement code to Generate START Condition
		pI2CHandle->pI2Cx->CR1 |= (0x1 << I2C_CR1_START);


	}

	return busystate;

}



/*********************************************************************
 * @fn      		  - void I2C_IRQ_ITConfig(uint8_t IRQNumber, uint8_t En_Di)
 *
 * @brief             - Enables/Disables the IRQ Reception line of the given irqnumber
 *
 * @param[in]         - uint8_t irqnumber
 * @param[in]         - uint8_t en_di
 *
 * @return            - void
 *
 * @Note              - Use it only for I2C peripheral
 */
void I2C_IRQ_ITConfig(uint8_t IRQNumber, uint8_t En_Di)					//CONFIGURES IRQ
{

	if(En_Di == ENABLE)
	{
		if(IRQNumber <= 31 )
		{
			//Set ISER0
			*NVIC_ISER0 |= (0x1 << IRQNumber);
		}else if(IRQNumber <= 63 && IRQNumber > 31)
		{
			//Set ISER1
			*NVIC_ISER1 |= (0x1 << (IRQNumber%32));
		}else if(IRQNumber <=95 && IRQNumber > 63)
		{
			//set ISER2
			*NVIC_ISER2 |= (0x1 << (IRQNumber%64));
		}else
		{
			if(IRQNumber <= 31)
			{
				//Set ICER0
				*NVIC_ICER0 |= (0x1 << (IRQNumber));
			}else if(IRQNumber <= 63 && IRQNumber > 31)
			{
				//Set ICER1
				*NVIC_ICER1 |= (0x1 << (IRQNumber%32));
			}else if(IRQNumber <= 95 && IRQNumber > 63)
			{
				//Set ICER2
				*NVIC_ICER2 |= (0X1 << (IRQNumber%64));
			}
		}
	}
}


/*********************************************************************
 * @fn      		  - void I2C_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority)
 *
 * @brief             - configures the priority of the given IRQNumber
 *
 * @param[in]         - uint8_t IRQNumber
 * @param[in]         - uint32_t IRQPriority
 *
 * @return            - void
 *
 * @Note              - Use only for I2C peripheral
 */
void I2C_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority)					// I2C PRIORITY HANDLER
{

	uint8_t iprx			= IRQNumber / 4;
	uint8_t ipr_section = IRQNumber % 4;

	uint8_t shift_amnt = ((8*ipr_section) + (8 - NO_PRIORITY_BITS));
	*( NVIC_IPR_BA + iprx )   |= (IRQPriority << shift_amnt);

}


/*********************************************************************
 * @fn      		  - void I2C_SlaveSendData(I2C_RegDef_t *pI2Cx, uint8_t data )
 *
 * @brief             - Sends the given data over the specified pI2Cx peripheral in slave mode
 *
 * @param[in]         - I2C_RegDef_t *pI2Cx
 * @param[in]         - uint8_t data
 *
 * @return            - void
 *
 * @Note              - NULL
 */
void I2C_SlaveSendData(I2C_RegDef_t *pI2Cx, uint8_t data )
{
	pI2Cx->DR = data;
}

/*********************************************************************
 * @fn      		  - uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2Cx)
 *
 * @brief             - Receives the data over the specified pI2Cx peripheral in slave mode
 *
 * @param[in]         - I2C_RegDef_t *pI2Cx
 *
 * @return            - uint8_t(the data received)
 *
 * @Note              - NULL
 */
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2Cx)
{
	return (uint8_t)pI2Cx->DR;
}

/*********************************************************************
 * @fn      		  - void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle)
 *
 * @brief             - Event Handler for the given I2C peripheral for both slave and master modes
 *
 * @param[in]         - I2C_Handle_t *pI2CHandle
 *
 * @return            - void
 *
 * @Note              - NULL
 */
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle)
{

	uint32_t temp1, temp2, temp3;

	//Interrupt handling for both master and slave mode of a device
	temp1 = pI2CHandle->pI2Cx->CR2 & (0x1 << I2C_CR2_ITEVTEN);
	temp2   = pI2CHandle->pI2Cx->CR2 & (0x1 << I2C_CR2_ITBUFEN);

	temp3  = pI2CHandle->pI2Cx->SR1 & (0x1 << I2C_SR1_SB);
	//1 Handle for interrupt generated by SB event
	//Note : SB flag is only applicable in Master mode
	if(temp1 && temp3)
	{
		//SB flag is set
		//Interrupt generated because of SB event
		//This block will not be executed in slave mode because for slave SB is always zero
		//In this block we have to execute the addres phase
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_Tx)
		{
			I2C_ExecuteAddrPhase(pI2CHandle->pI2Cx, pI2CHandle->DevAddr, WRITE);
			//printf("Address enabled\n");
		}else
		{
			I2C_ExecuteAddrPhase(pI2CHandle->pI2Cx, pI2CHandle->DevAddr,READ);
			//printf("Address enabled\n");
		}

	}

	temp3 = pI2CHandle->pI2Cx->SR1 & (0x1 << I2C_SR1_ADDR);
	//2. Handle for interrupt generated by ADDR event
	//Note : When MASTER mode : ADDRESS is sent
	//			  When SLAVE mode : ADDRESS matched with own address
	if(temp1 && temp3)
	{
		//printf("Address CLeared\n");
		I2C_ClearADDRFlag(pI2CHandle);
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & (0x1 << I2C_SR1_BTF);
	//3. Handle for interrupt generated by BTF(Byte Transfer Finished) event
	if(temp1 && temp3)
	{
		//BTF flag is set
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_Tx)
		{
			//MASTER in Tx mode
			if(pI2CHandle->pI2Cx->SR1 & (0x1 << I2C_SR1_TxE))
			{
				//BTF && TXE == 1
				if(pI2CHandle->TxLen == 0)
				{
					//1. Generate the STOP condition
					if(pI2CHandle->Sr == I2C_DISABLE_SR)
						pI2CHandle->pI2Cx->CR1 |= (0x1 << I2C_CR1_STOP );
					//printf("STOP generated\n");

					//2. Reset all the member elements of the handle structure
					I2C_CloseSendData(pI2CHandle);

					//3. Notify the application about transmission complete
					I2C_AppEventCallback(pI2CHandle, I2C_EV_Tx_COMPLETE);
				}
 			}
		}else
		{
			//MASTER in Rx mode
			;
		}
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & (0x1 << I2C_SR1_STOPF);
	//4. Handle for interrupt generated by STOPF event
	//Note : Stop detection flag is applicable only slave mode, For MASTER this flag will
	if(temp1 && temp3)
	{
		//STOPF flag is set
		//Clear the STOPF ; 1) Read SR1 2) Write to CR1

			//printf("STOPF detected\n");
			temp3 |= pI2CHandle->pI2Cx->SR1;
		pI2CHandle->pI2Cx->CR1 |= 0x0;

		//Notify the application that STOP is detected
		I2C_AppEventCallback(pI2CHandle, I2C_EV_STOP);
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & (0x1 << I2C_SR1_TxE);
	//5. Handle for interrupt generated by TXE event
	if(temp1 && temp2 && temp3)
	{
		if(pI2CHandle->pI2Cx->SR2 & ( 0x1 << I2C_SR2_MSL ))
		{
			 //TXE flag is set
			//We have to SEND data here
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_Tx)
			{
				if(pI2CHandle->TxLen > 0)
				{
					//1. Load data in DR
					pI2CHandle->pI2Cx->DR = *(pI2CHandle->pTxBuffer);

					//2. Decrement the TxLen
					pI2CHandle->TxLen--;

					//3. Increment the buffer Addr
					pI2CHandle->pTxBuffer++;
					//printf("TXE occured Send data\n");
				}
			}
		}else
		{
			//SLAVE
			if(pI2CHandle->pI2Cx->SR2 & (0x1 << I2C_SR2_TRA) )
			{
			//printf("SLAVE data request\n");
				I2C_AppEventCallback(pI2CHandle, I2C_EV_DATA_REQUEST);
			//	if(cmnd_code == 0xff ) cmnd_code  = 0x52;

			}
		}
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & (0x1 << I2C_SR1_RxNE);
	//	6. Handle for interrupt generated by RXNE
	if(temp1 && temp2 && temp3)
	{
		//RXNE flag is set
		if(pI2CHandle->pI2Cx->SR2 &(0x1 << I2C_SR2_MSL))
		{
			//Device is in Rx mode
			if( pI2CHandle->TxRxState == I2C_BUSY_IN_Rx  )
			{
				//Device is MASTER
				if( pI2CHandle->RxLen == 1)
				{

					// Read data in to buffer
					pI2CHandle->RxSize  |= pI2CHandle->pI2Cx->DR;

					pI2CHandle->RxLen--;
			//printf("RXNE len == 1\n");
				}

				if(pI2CHandle->RxLen > 1)
				{
					//Read the data until Len becomes Zero
						if(pI2CHandle->RxLen == 2)
						{
							//Clear the ACK bit
							pI2CHandle->pI2Cx->CR1 &= ~(0x1 << I2C_CR1_ACK);
						}

						//Read the data from data reg in to buffer
						*(pI2CHandle->pRxBuffer) = pI2CHandle->pI2Cx->DR;

						//Increment the buffer Addr
						pI2CHandle->pRxBuffer++;

						//Decrement RxLen
						pI2CHandle->RxLen--;
						//printf("RXNE len > 1\n");
				}

				if(pI2CHandle->RxLen == 0)
				{
					//Close the I2C data reception and notify the application

					//printf("RXLEN == 0 ; Close reception\n");
					//1.STOP condition needs to be generated
					if(pI2CHandle->Sr == I2C_DISABLE_SR)
						pI2CHandle->pI2Cx->CR1 |= (0x1 << I2C_CR1_STOP);

					//2.Close the I2C Rx
					I2C_CloseReceiveData(pI2CHandle);

					//3.Notify the application
					I2C_AppEventCallback(pI2CHandle, I2C_EV_Rx_COMPLETE);
  				}

			}
		}else
		{
			//SLAVE
			if( !(pI2CHandle->pI2Cx->SR2 & (0x1 << I2C_SR2_TRA)) )
			{
			//printf("SLAVE DATA BYtes receive \n");
				I2C_AppEventCallback(pI2CHandle, I2C_EV_DATA_RECEIVE);
			}
		}
	}

}

/*********************************************************************
 * @fn      		  - void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle)
 *
 * @brief             - Closes the communication with the device communicating at the given
 * 						I2C peripheral
 *
 * @param[in]         - I2C_Handle_t *pI2CHandle
 *
 * @return            - void
 *
 * @Note              - Used in both Slave && Master modes
 */
void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle)
{
	//Implement the code to disable ITBUFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~(0x1 << I2C_CR2_ITBUFEN);

	//Implement the code to disable ITEVFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~(0x1 << I2C_CR2_ITEVTEN);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pRxBuffer = NULL;
	pI2CHandle->RxLen = 0;

	if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_EN)
	{
		pI2CHandle->pI2Cx->CR1 |= (0x1 << I2C_CR1_ACK);
	}

}

/*********************************************************************
 * @fn      		  - void I2C_CloseSendData(I2C_Handle_t *pI2CHandle)
 *
 * @brief             - Closes the communication with the device communicating at the given
 * 						I2C peripheral
 *
 * @param[in]         - I2C_Handle_t *pI2CHandle
 *
 * @return            - void
 *
 * @Note              - Used in both Slave && Master modes
 */
void I2C_CloseSendData(I2C_Handle_t *pI2CHandle)
{
	//Implement the code to disable ITBUFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~(0x1 << I2C_CR2_ITBUFEN);

	//Implement the code to disable ITEVFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~(0x1 << I2C_CR2_ITEVTEN);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pTxBuffer = NULL;
	pI2CHandle->TxLen = 0;

	if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_EN)
	{
		pI2CHandle->pI2Cx->CR1 |= (0x1 << I2C_CR1_ACK);
	}

}

/*********************************************************************
 * @fn      		  - void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle)
 *
 * @brief             - Error IRQ handler
 *
 * @param[in]         - I2C_Handle_t *pI2CHandle
 *
 * @return            - void
 *
 * @Note              - Calls appropriate Callback( I2C_AppEventCallbac() ) to the the USER application
 * 						Informing about the necessary errors/events
 */
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle)
{

	uint32_t temp1,temp2;

    //Know the status of  ITERREN control bit in the CR2
	temp2 = (pI2CHandle->pI2Cx->CR2) & ( 1 << I2C_CR2_ITERREN);


/***********************Check for Bus error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1<< I2C_SR1_BERR);
	if(temp1  && temp2 )
	{
		//This is Bus error

		//Implement the code to clear the buss error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_BERR);

		//Implement the code to notify the application about the error
	   I2C_AppEventCallback(pI2CHandle,I2C_ERROR_BERR);
	}

/***********************Check for arbitration lost error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_ARLO );
	if(temp1  && temp2)
	{
		//This is arbitration lost error

		//Implement the code to clear the arbitration lost error flag
		pI2CHandle->pI2Cx->SR1 &= ~(0x1 << I2C_SR1_ARLO);

		//Implement the code to notify the application about the error
		I2C_AppEventCallback(pI2CHandle, I2C_ERROR_ARLO);
	}

/***********************Check for ACK failure  error************************************/

	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_AF);
	if(temp1  && temp2)
	{
		//This is ACK failure error

	    //Implement the code to clear the ACK failure error flag
		pI2CHandle->pI2Cx->SR1 &= ~(0x1 << I2C_SR1_AF);

		//Implement the code to notify the application about the error
		I2C_AppEventCallback(pI2CHandle, I2C_ERROR_AF);
	}

/***********************Check for Overrun/underrun error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_OVR);
	if(temp1  && temp2)
	{
		//This is Overrun/underrun

	    //Implement the code to clear the Overrun/underrun error flag
		pI2CHandle->pI2Cx->SR1 &= ~(0x1 << I2C_SR1_OVR);

		//Implement the code to notify the application about the error
		I2C_AppEventCallback(pI2CHandle, I2C_ERROR_OVR);
	}

/***********************Check for Time out error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_TIMEOUT);
	if(temp1  && temp2)
	{
		//This is Time out error

	    //Implement the code to clear the Time out error flag
		pI2CHandle->pI2Cx->SR1 &= ~(0x1 << I2C_SR1_TIMEOUT);

		//Implement the code to notify the application about the error
		I2C_AppEventCallback(pI2CHandle, I2C_ERROR_TIMEOUT);
	}

}


/*********************************************************************
 * @fn      		  - void I2C_SlaveEnDiCallBackEvents(I2C_RegDef_t *pI2Cx, uint8_t ENDI)
 *
 * @brief             - Enables/Disables Error/Event callback to the USER application when in Slave mode
 *
 * @param[in]         - I2C_RegDef_t *pI2Cx
 * @param[in]         - uint8_t ENDI
 *
 * @return            - void
 *
 * @Note              - NULL
 */
void I2C_SlaveEnDiCallBackEvents(I2C_RegDef_t *pI2Cx, uint8_t ENDI)
{
	if(ENDI == ENABLE)
	{
		pI2Cx->CR2 |= (0x1 << I2C_CR2_ITEVTEN);
		pI2Cx->CR2 |= (0x1 << I2C_CR2_ITBUFEN);
		pI2Cx->CR2 |= (0x1 << I2C_CR2_ITERREN);

		//printf("Slave Enabled\n");

	}else
	{
		pI2Cx->CR2 &= ~(0x1 << I2C_CR2_ITEVTEN);
		pI2Cx->CR2 &= ~(0x1 << I2C_CR2_ITBUFEN);
		pI2Cx->CR2 &= ~(0x1 << I2C_CR2_ITERREN);

	}
}

/*********************************************************************
 * @fn      		  - __weak void I2C_AppEventCallback(I2C_Handle_t *pI2CHandle, uint8_t I2C_Event)
 *
 * @brief             - This is weak implementation of the callback function; Do not delete this:
 * 						Copy the function to USER code and remove the __weak attribute
 *
 * @param[in]         - I2C_Handle_t *pI2CHandle, uint8_t I2C_Event
 *
 * @return            - void
 *
 * @Note              - DO NOT REMOVE THIS FROM HERE
 */
__weak void I2C_AppEventCallback(I2C_Handle_t *pI2CHandle, uint8_t I2C_Event)
{

}