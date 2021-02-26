/*
 * stm32f445xx_spi.h
 *
 *  Created on: Feb 1, 2021
 *      Author: ark
 */

#ifndef STM32F446XX_SPI_H_
#define STM32F446XX_SPI_H_

#include<stdint.h>
#include"stm32f446xx.h"
#include<iostream>


class SPI_Config_t:{
	/*
	 * CONFIG STRUCT FOR SPIx PERIPHERAL
	 */

private:
	uint8_t SPI_DeviceMode;						//CAN BE ANYONE OF @SPI_DeviceMode
	uint8_t SPI_BusConfig;						//CAN BE ANYONE OF @SPI_BusConfig
	uint8_t SPI_SclkSpeed;						//CAN BE ANYONE OF @SPI_SclkSpeed
	uint8_t SPI_DFF;							//CAN BE ANYONE OF @SPI_DFF
	uint8_t SPI_CPOL;							//CAN BE ANYONE OF @SPI_CPOL
	uint8_t SPI_CPHA;							//CAN BE ANYONE OF @SPI_CPHA
	uint8_t SPI_SSM;							//CAN BE ANYONE OF @SPI_SSM


SPI_Config_t():SPI_DeviceMode(0),SPI_BusConfig(0),SPI_SclkSpeed(0),SPI_DFF(0),SPI_CPOL(0),SPI_CPHA(0),SPI_SSM(0){};


protected:
  void SPI_Config(uint8_t devicemode=0,uint8_t busconfig=0,uint8_t sclksped=0,uint8_t dff=0,uint8_t cpol=0,uint8_t cpha=0,uint8_t ssm=0){
    
	 SPI_DeviceMode=devicemode;						
	 SPI_BusConfig=buscongfig;					
	 SPI_SclkSpeed=sclksped;					
	 SPI_DFF=dff;
	 SPI_CPOL=cpol;	
	 SPI_CPHA=cpha;	
	 SPI_SSM=ssm;
  };

}

class SPI_Handle_t: public SPI_Config_t
{
	/*
	 * HANDLE STRUCT FOR SPIx PERIPHERAL
	 */

public:
	SPI_RegDef_t *pSPIx;						//HOLDS THE ADDRESS OF THE SPIx(x : 1,2,3) PERIPHERAL ADDRESS
	void SPI_Handle(uint8_t S1,uint8_t S2,uint8_t S3,uint8_t S4,uint8_t S5,uint8_t S6,uint8_t S7){
		SPI_Config(S1,S2,S3,S4,S5,S6,S7);
	};
	uint8_t		 *pTxBuffer;					//STORES THE APPLICATION Tx BUFFER ADDRESS
	uint8_t		 *pRxBuffer;					//STORES THE APPLICATION Rx BUFFER ADDRESS
	uint32_t 	  TxLen;
	uint32_t 	  RxLen;
	uint8_t		  TxState;
	uint8_t		  RxState;

}

/************************************************************************************************************************************
 * 																		 MACROS FOR THE DRIVER																						*
 * 															TO BE USED WITH SPI_CONFIG STRUCT																			*
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

#define SPI_TXE_FLAG					(0x1 << SPI_SR_TXE)
#define SPI_RXNE_FLAG					(0x1 << SPI_SR_RXNE)
#define SPI_BUSY_FLAG					(0x1 << SPI_SR_BSY)
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
