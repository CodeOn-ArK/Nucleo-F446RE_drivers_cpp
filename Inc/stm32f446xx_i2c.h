/*
 * stm32f446xx_i2c.h
 *
 *  Created on: 07-Feb-2021
 *      Author: ark
 */

#ifndef STM32F446XX_I2C_H_
#define STM32F446XX_I2C_H_

#include<stdint.h>
#include"stm32f446xx.h"


class I2C_Config_t:{


  private:
	__vo uint8_t I2C_SCLSpeed	;							//CAN		BE 	ANYONE	OF	@I2C_SCLSpeed
	__vo uint8_t I2C_DeviceAddress	;							//CAN		BE 	ANYONE	OF	@I2C_DeviceAddress
	__vo uint8_t I2C_ACKControl	;							//CAN		BE 	ANYONE	OF	@I2C_ACKControl
	__vo uint8_t I2C_FMDutyCycle	;							//CAN		BE 	ANYONE	OF	@I2C_FMDutyCycle

I2C_Config_t(): I2C_SCLSpeed(0),I2C_DeviceAddress(0),I2C_ACKControl(0),I2C_FMDutyCycle(0){};

protected:
  void I2C_Config(uint8_t speed=0,uint8_t deviceaddr=0, uint8_t ackcontrol=0,uint8_t fmdutycycle=0)
  {
      I2C_SCLSpeed=speed;
	  I2C_DeviceAddress= deviceaddr;
	  I2C_DeviceAddress= ackcontrol;
	  I2C_FMDutyCycle=fmdutycycle;

  }


};


/*
 * HANDLE		STRUCTURE		FOR 		I2C 		PERIPHERAL
 */
class I2C_Handle_t:public I2C_Config_t
{


public:
	I2C_RegDef_t	*pI2Cx;
	void I2C_handle(uint8_t I1,uint8_t I2,uint8_t I3,uint8_t I4){
		I2C_Config(I1,I2,I3,I4);
	}
	uint8_t			*pTxBuffer;			// !<	  To store the app. Tx buffer address 	>
	uint8_t			*pRxBuffer;			// !<	  To store the app. Rx buffer addr	>
	uint32_t			TxLen;					// !<	  To store Tx Len	>
	uint32_t			RxLen;					// !<	  To store Rx Len	>
	uint8_t			TxRxState;			// !<	  To store communication state @I2C_application_state	>
	uint8_t			DevAddr;				// !<	  To store Slave/device	addr	>
	uint32_t			RxSize;					// !<	  To Store Rx size >
	uint8_t			Sr	;						// !<	  To store Repeated start value	>


};

/************************************************************************************************************************************
 * 																			 MACROS FOR THE DRIVER																								*
 * 																TO BE USED WITH I2C_CONFIG STRUCT																						*
 ************************************************************************************************************************************/
/*
 * @I2C_SCLSpeed
 */
#define I2C_SCL_SPEED_SM		0x186A0U
#define I2C_SCL_SPEED_FM2K	0x30D40U
#define I2C_SCL_SPEED_FM4K	0x61A80U

/*
 *@ I2C_DeviceAddress
 */
//DEPENDS		UPON 		THE 		USER	PERMITABLE		RANGE		0-127


/*
 * @I2C_ACKControl
 */
#define I2C_ACK_EN						1
#define I2C_ACK_DI						0

/*
 * @I2C_FMDutyCycle
 */
#define  I2C_FM_DUTY_2				0
#define  I2C_FM_DUTY_16_9			1

/*
 * @I2C_application_state
 */
#define	I2C_READY						0
#define	I2C_BUSY_IN_Rx				1
#define 	I2C_BUSY_IN_Tx				2


/*
 * Status register Flags
 */

#define I2C_FLAG_SR1_SB				0
#define I2C_FLAG_SR1_ADDR			1
#define I2C_FLAG_SR1_BTF			2
#define I2C_FLAG_SR1_ADD10		3
#define I2C_FLAG_SR1_STOPF		4
#define I2C_FLAG_SR1_RxNE			6
#define I2C_FLAG_SR1_TxE				7
#define I2C_FLAG_SR1_BERR			8
#define I2C_FLAG_SR1_ARLO			9
#define I2C_FLAG_SR1_AF				10
#define I2C_FLAG_SR1_OVR			11
#define I2C_FLAG_SR1_PECERR		12
#define I2C_FLAG_SR1_TIMEOUT	14
#define I2C_FLAG_SR1_SMBALERT	15



#define I2C_FLAG_SR2_MSL			0
#define I2C_FLAG_SR2_BSY			1
#define I2C_FLAG_SR2_TRA			2
#define I2C_FLAG_SR2_GENCALL	4
#define I2C_FLAG_SR2_SMBDEFAULT 	5
#define I2C_FLAG_SR2_SMBHOST 6
#define I2C_FLAG_SR2_DUALF		7
#define I2C_FLAG_SR2_PEC			8

/*
 * I2C EVENT MACROS
 */

#define I2C_EV_Tx_COMPLETE		0
#define I2C_EV_Rx_COMPLETE		1
#define I2C_EV_STOP						2

#define I2C_DISABLE_SR				1
/************************************************************************************************************************************
 * 												APIS SUPPORTED BY THIS DRIVER																												*
 * 											FOR MORE INFO CHECK THE FUNCTION DESCP																								*
 ************************************************************************************************************************************/

/*
 * I2C CLOCK CONTROL
 */

void I2C_PeriClkCntrl(I2C_RegDef_t *pI2Cx, uint8_t En_Di);

/*
 * INIT && DE-INIT
 */

void I2C_Init(I2C_Handle_t *pI2CHandle);									//INITIALIZES I2C PORT
void I2C_DeInit(I2C_RegDef_t *pI2Cx);										//DEINITIALIZES I2C PORT

/*
 *BLOCKING		 DATA TRANSMIT / RECIEVE
 */

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTXBuffer, uint32_t Len, uint8_t SlaveAddr);
void I2C_MasterRecieveData(I2C_Handle_t *pI2CHandle, uint8_t *pRXBuffer, uint32_t Len, uint8_t SlaveAddr);


/*
 * INTERRUPT	DATA TRANSMIT / RECIEVE
 */

uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTXBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr);
uint8_t I2C_MasterRecieveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRXBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr);

void I2C_CloseSendData(I2C_Handle_t *pI2CHandle);
void I2C_CloseRecieveData(I2C_Handle_t *pI2CHandle);
/*
 * IRQ CONFIG && ISR HANDLING
 */


void I2C_IRQ_ITConfig(uint8_t IRQNumber, uint8_t En_Di);					//CONFIGURES IRQ
void I2C_IRQConfig(uint8_t IRQNumber,uint32_t IRQPriority);					// I2C PRIORITY HANDLER

void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle);
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle);

/*
 * ADDN APIs
 */
uint32_t RCC_GetClkVal(void);
void I2C_Enable(I2C_RegDef_t *I2C_ENDI, uint8_t EN_DI);
uint8_t I2C_GetFagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName);
/*
 * APPLICATION CALLBACK
 */

void I2C_AppEventCallback(I2C_Handle_t *, uint8_t);



#endif /* STM32F446XX_I2C_H_ */