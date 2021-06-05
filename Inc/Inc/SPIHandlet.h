/*
 * SPIHandlet.h
 *
 *  Created on: 01-Jun-2021
 *      Author: root
 */

#ifndef INC_SPIHANDLET_H_
#define INC_SPIHANDLET_H_

#include "stm32f446xx.h"


#include<stdint.h>


class SPI_Config_t {
	/*
	 * CONFIG STRUCT FOR SPIx PERIPHERAL
	 */

public:
	uint8_t SPI_DeviceMode;						//CAN BE ANYONE OF @SPI_DeviceMode
	uint8_t SPI_BusConfig;						//CAN BE ANYONE OF @SPI_BusConfig
	uint8_t SPI_SclkSpeed;						//CAN BE ANYONE OF @SPI_SclkSpeed
	uint8_t SPI_DFF;							//CAN BE ANYONE OF @SPI_DFF
	uint8_t SPI_CPOL;							//CAN BE ANYONE OF @SPI_CPOL
	uint8_t SPI_CPHA;							//CAN BE ANYONE OF @SPI_CPHA
	uint8_t SPI_SSM;							//CAN BE ANYONE OF @SPI_SSM

	SPI_Config_t();
	virtual ~SPI_Config_t();

};

class SPI_Handle_t: public SPI_Config_t{
	/*
	 * HANDLE STRUCT FOR SPIx PERIPHERAL
	 */

public:
	SPI_RegDef_t *pSPIx;						//HOLDS THE ADDRESS OF THE SPIx(x : 1,2,3) PERIPHERAL ADDRESS
	SPI_Config_t SPIConfig;
	uint8_t		 *pTxBuffer;					//STORES THE APPLICATION Tx BUFFER ADDRESS
	uint8_t		 *pRxBuffer;					//STORES THE APPLICATION Rx BUFFER ADDRESS
	uint32_t 	  TxLen;
	uint32_t 	  RxLen;
	uint8_t		  TxState;
	uint8_t		  RxState;

	SPI_Handle_t();
	virtual ~SPI_Handle_t();


};

/************************************************************************************************************************************
 * 														MACROS FOR THE DRIVER																						*
 * 												   TO BE USED WITH SPI_CONFIG STRUCT																			*
 ************************************************************************************************************************************/
//@SPI_DeviceMode

#define SPI_MODE_SLAVE					0
#define SPI_MODE_MASTER					1

//@SPI_BusConfig

#define SPI_BUS_CONFIG_FD				1
#define SPI_BUS_CONFIG_HD				2
#define SPI_BUS_CONFIG_SIMPLEX_Rx		3

//@SPI_SclkSpeed

#define SPI_SCLK_SPEED_DIV2				0
#define SPI_SCLK_SPEED_DIV4				1
#define SPI_SCLK_SPEED_DIV8				2
#define SPI_SCLK_SPEED_DIV16			3
#define SPI_SCLK_SPEED_DIV32			4
#define SPI_SCLK_SPEED_DIV64			5
#define SPI_SCLK_SPEED_DIV128			6
#define SPI_SCLK_SPEED_DIV256			7

//@SPI_DFF

#define SPI_DFF_8						0
#define SPI_DFF_16						1

//@SPI_CPOL

#define SPI_CPOL_LOW					0
#define SPI_CPOL_HIGH					1

//@SPI_CPHA

#define SPI_CPHA_LOW					0
#define SPI_CPHA_HIGH					1

//@SPI_SSM

#define SPI_SSM_DI						0
#define SPI_SSM_EN						1

//FLAG MACROS

#define SPI_TXE_FLAG					SPI_SR_TXE
#define SPI_RXNE_FLAG					SPI_SR_RXNE
#define SPI_BUSY_FLAG					SPI_SR_BSY
#define SPIEN							6
#define SSIEN							8
#define SSOEEN							2

//SPI APPLICATION STATES

#define SPI_READY						0
#define SPI_BUSY_Rx						1
#define SPI_BUSY_Tx						2

//SPI APPLICATION EVENTS

#define SPI_EVENT_TX_CMPLT				1
#define SPI_EVENT_RX_CMPLT				2
#define SPI_EVENT_OVR_ERR				3

/************************************************************************************************************************************
 * 												APIS SUPPORTED BY THIS DRIVER																									*
 * 											FOR MORE INFO CHECK THE FUNCTION DESCP																			*
 ************************************************************************************************************************************/

/*
 * SPI CLOCK CONTROL
 */

void SPI_PeriClkCntrl(SPI_RegDef_t *pSPIx, uint8_t En_Di);

/*
 * INIT && DE-INIT
 */

void SPI_Init(SPI_Handle_t *pSPIHandle);									//INITIALIZES SPI PORT
void SPI_DeInit(SPI_RegDef_t *pSPIx);										//DEINITIALIZES SPI PORT

/*
 * DATA TRANSMIT / RECIEVE
 */

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTXBuffer, uint32_t Len);
void SPI_RecieveData(SPI_RegDef_t *pSPIx, uint8_t *pRXBuffer, uint32_t Len);

uint8_t SPI_SendData_IT(SPI_Handle_t *pSPIHandle, uint8_t *pTXBuffer, uint32_t Len);
uint8_t SPI_RecieveData_IT(SPI_Handle_t *pSPIHandle, uint8_t *pRXBuffer, uint32_t Len);
/*
 * IRQ CONFIG && ISR HANDLING
 */


void SPI_IRQ_ITConfig(uint8_t IRQNumber, uint8_t En_Di);					//CONFIGURES IRQ
void SPI_IRQHandling(SPI_Handle_t *pHandle);								//HANDLER CODE FOR ISR
void SPI_IRQConfig(uint8_t IRQNumber,uint32_t IRQPriority);					// SPI PRIORITY HANDLER

/*
 * ADDN APIs
 */

void SPI_Enable(SPI_RegDef_t *SPI_ENDI, uint8_t EN_DI);
uint8_t SPI_GetFagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName);
void SSOE_Config(SPI_RegDef_t *SPI_ENDI, uint8_t EN_DI);
void SSI_Config(SPI_RegDef_t *SPI_ENDI, uint8_t EN_DI);

void SPI_ClearOVR(SPI_RegDef_t *);
void SPI_AbortTx(SPI_Handle_t *);
void SPI_AbortRx(SPI_Handle_t *);

/*
 * APPLICATION CALLBACK
 */

void SPI_AppEventCallback(SPI_Handle_t *, uint8_t);

#endif /* STM32F446XX_SPI_H_ */
