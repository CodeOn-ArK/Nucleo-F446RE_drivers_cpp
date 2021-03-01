/*
 * stm32f445xx_gpio.h
 *
 *  Created on: Feb 1, 2021
 *      Author: ark
 */

#ifndef STM32F446XX_GPIO_H_
#define STM32F446XX_GPIO_H_

#include<stdint.h>
#include"stm32f446xx.h"


class GPIO_PinConfig_t:{

	   private:
             __vo uint8_t GPIO_PinNumber;							// CAN BE ANYONE OF @GPIO_PIN_NUMBERS
	         __vo uint8_t GPIO_PinMode;								// CAN BE ANYONE OF @GPIO_MODES
	         __vo uint8_t GPIO_PinSpeed;								// CAN BE ANYONE OF @GPIO_SPEED
	         __vo uint8_t GPIO_PinPuPdControl;						// CAN BE ANYONE OF @GPIO_PULL_UP_PULL_DOWN
	         __vo uint8_t GPIO_PinOPType;							// CAN BE ANYONE OF @GPIO_OUTPUT_TYPE
	         __vo uint8_t GPIO_PinAltFunMode;



GPIO_PinConfig_t():  GPIO_PinNumber(0), GPIO_PinMode(0),GPIO_PinSpeed(GPIO_SPEED_LOW),GPIO_PinPuPdControl(GPIO_NO_PUPD), GPIO_PinOPType(GPIO_OP_TYPE_PP), GPIO_PinAltFunMode(GPIO_ALTFN_OP){}; 

protected:
void set_Pin(uint8_t pin=0,uint8_t pinmode=0, uint8_t pinaltfunmode=0) {
   GPIO_PinNumber= pin;
   GPIO_PinMode=pinmode;
   GPIO_PinAltFunMode=pinaltfunmode;
}



	
};



/*
 * THIS IS A HANDLE STRUCTURE FOR GPIO PIN
 */
class GPIO_handle_t: public GPIO_PinConfig_t
{

public:
	GPIO_regdef_t *pGPIOx;	//THIS HOLDS THE BASE ADDR OF THE GPIO PORT TO WHICH THE PIN BELONGS
	void set_Pinvalues(uint8_t p1,uint8_t p2,uint8_t p3){
    set_Pin(p1,p2,p3);
}

   //GPIO_PinConfig_t GPIO_PinConfig;//THIS HOLDS GPIO PIN CONFIG SETTINGS

};

/************************************************************************************************************************************
 * 												 MACROS FOR THE DRIVER																*
 * 											TO BE USED WITH GPIO_CONFIG STRUCT														*
 ************************************************************************************************************************************/
//	 @GPIO_PIN_NUMBERS		GPIO PIN SPECIFIC MACROS
#define GPIO_PIN_0				0
#define GPIO_PIN_1				1
#define GPIO_PIN_2				2
#define GPIO_PIN_3				3
#define GPIO_PIN_4				4
#define GPIO_PIN_5				5
#define GPIO_PIN_6				6
#define GPIO_PIN_7				7
#define GPIO_PIN_8				8
#define GPIO_PIN_9				9
#define GPIO_PIN_10				10
#define GPIO_PIN_11				11
#define GPIO_PIN_12				12
#define GPIO_PIN_13				13
#define GPIO_PIN_14 			14
#define GPIO_PIN_15				15

//   @GPIO_MODES 		    GPIO MODE SPECIFIC MACORS
#define	GPIO_MODE_IN			0			//GPIO INPUT MODE
#define	GPIO_MODE_OUT			1			//GPIO OUTPUT MODE
#define GPIO_ALT_FN				2			//GPIO ALTERNATE FUNCTION
#define	GPIO_MODE_ANALOG		3			//GPIO ANALOG MODE
#define GPIO_MODE_IT_FT			4			//GPIO INTERRUPT FALLING EDGE TRIGGER
#define GPIO_MODE_IT_RT			5			//GPIO INTERRPUT RISING EDGE TRIGGER
#define	GPIO_MODE_IT_RFT		6			//GPIO INTERRUPT RISING AND FALLING EDGE TRIGGER


//   @GPIO_OUTPUT_TYPE      GPIO OUTPUT TYPES
#define GPIO_OP_TYPE_PP			0			//GPIO OUTPUT TYPE PUSH-PULL
#define GPIO_OP_TYPE_OD			1			//GPIO OUTPUT TYPE OPEN DRAIN


//  @GPIO_SPEED				GPIO SPEED TYPES
#define GPIO_SPEED_LOW			0			//LOW SPEED
#define GPIO_SPEED_MEDIUM		1			//MEDIUM SPEED
#define GPIO_SPEED_FAST			2			//FAST SPEED
#define GPIO_SPEED_HIGH			3			//HIGH SPEED


//  @GPIO_PULL_UP_PULL_DOWN GPIO PULL UP / PULL DOWN MACROS
#define GPIO_NO_PUPD			0			//NO PULL UP OR PULL DOWN (INTERNAL)
#define GPIO_PIN_PU				1			//ENABLE PULL UP ON THE PIN
#define GPIO_PIN_PD				2			//ENABLE PULL DOWN RESISTOR


/************************************************************************************************************************************
 * 												APIS SUPPORTED BY THIS DRIVER														*
 * 											FOR MORE INFO CHECK THE FUNCTION DESCP													*
 ************************************************************************************************************************************/

void GPIO_PeriClkCntrl(GPIO_regdef_t *pGPIOx, uint8_t En_Di);				//CONTROLS GPIO CLOCK

//TODO: WE WILL REFACTOR THE FUNCTIONS LATER;
void GPIO_Init(GPIO_handle_t *pGPIOHandle);									//INITIALIZES GPIO PORT
void GPIO_DeInit(GPIO_regdef_t *pGPIOx);									//DEINITIALIZES GPIO PORT


uint8_t GPIO_ReadIPin(GPIO_regdef_t *pGPIOx, uint8_t Pin_No);				//READS INPUT FROM A PIN OF A GPIO
uint16_t GPIO_ReadIPort(GPIO_regdef_t *pGPIOx);								//READS INPUT FROM A PORT OF A GPIO
void GPIO_WriteOPin(GPIO_regdef_t *pGPIOx, uint8_t Pin_No, uint8_t value);	//WRITES TO PIN OF A GPIO
void GPIO_WriteOPort(GPIO_regdef_t *pGPIOx, uint16_t value);				//WRITES TO PORT OF A GPIO
void GPIO_ToggleOPin(GPIO_regdef_t *pGPIOx, uint8_t Pin_No);				//TOGGLES A PIN A GIVEN GPIO


void GPIO_IRQ_ITConfig(uint8_t IRQNumber, uint8_t En_Di);					//CONFIGURES IRQ
void GPIO_IRQHandling(uint8_t Pin_No);										//HANDLER CODE FOR ISR
void GPIO_IRQConfig(uint8_t IRQNumber,uint32_t IRQPriority);				// GPIO PRIORITY HANDLER













#endif /* STM32F446XX_GPIO_H_ */