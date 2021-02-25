/*
 * stm32f446xx_gpio.cpp
 *
 *  Created on: Feb 1, 2021
 *      Author: ark
 */


#include "stm32f446xx_gpio_cpp.h"


/********************************************************************************
 * 							DOCUMENTATION
 * @fn		: GPIO_PeriClkCntrl
 *
 * @brief	: this func ENABLES or DISABLES for the given GPIO peripheral clock. The address of the GPIO port is sent to it
 *
 * @param	: base addr of gpio peripheral
 * @param	: ENABLE or DISABLE macro
 * @param	:
 *
 * @return  : void
 *
 * @Note	: none
 */
void GPIO_PeriClkCntrl(GPIO_regdef_t *pGPIOx, uint8_t En_Di)
{
	if(En_Di == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_CLK_EN();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_CLK_EN();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_CLK_EN();
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_CLK_EN();
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_CLK_EN();
		}
		else if(pGPIOx == GPIOF)
		{
			GPIOF_CLK_EN();
		}
		else if(pGPIOx == GPIOG)
		{
			GPIOG_CLK_EN();
		}
		else if(pGPIOx == GPIOH)
		{
			GPIOH_CLK_EN();
		}
	}

	if(En_Di == DISABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_CLK_DI();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_CLK_DI();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_CLK_DI();
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_CLK_DI();
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_CLK_DI();
		}
		else if(pGPIOx == GPIOF)
		{
			GPIOF_CLK_DI();
		}
		else if(pGPIOx == GPIOG)
		{
			GPIOG_CLK_DI();
		}
		else if(pGPIOx == GPIOH)
		{
			GPIOH_CLK_DI();
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
void GPIO_Init(GPIO_handle_t *pGPIOHandle)
{
	uint32_t temp = 0;

	//CONFIG RCC FOR THE GPIO PORT
	GPIO_PeriClkCntrl(pGPIOHandle->pGPIOx, ENABLE);

	// CONFIGURE THE MODE OF GPIO PIN
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= 3)
	{

		temp = ( pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER |= temp;

	}
	else
	{
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			//CONFIGURE THE FTSR (FALLING TRIGGER SELECTION REGISTER)
			EXTI->FTSR |= (0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//CLEAR RTSR
			EXTI->RTSR &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			//CONFIGURE THE RTSR (RISING TRIGGER SELECTION REGISTER
			EXTI->RTSR |= (0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//CLEAR FTSR
			EXTI->FTSR &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{
			//CONFIGURE BOTH RTSR AND FTSRpGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR |= (0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR |= (0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		//CONFIGURE THE GPIO PORT SELECTION IN SYSCFG_EXTICR
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber/2;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber%2;

		uint8_t portcode = GPIO_BA_CODE(pGPIOHandle->pGPIOx);

		SYSCFG_CLK_EN();
		SYSCFG->EXTICR[temp1] |= ( portcode << 4*temp2 );


		//ENABLE THE EXTI INTERRUPT DELIVERY USING IMR
		EXTI->IMR |= (0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

	}

	temp = 0;
	// CONFIGURE THE SPEED
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;

	temp = 0;
	// CONFIGURE THE PUPD SETTINGS
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR |= temp;

	temp = 0;
	// CONFIGURE THE OPTYPE
	pGPIOHandle->pGPIOx->OTYPER |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));

	temp = 0;
	// CONFIGURE THE ALTERNATE FUNCTIONALITY OF THE PIN
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_ALT_FN)
	{
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber <= 3)
		{
			temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4)));
			pGPIOHandle->pGPIOx->AFR[0] |= temp;
		}
		else if((pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber <= 7) && (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber > 3))
		{
			temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (16 + 4 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4)));
			pGPIOHandle->pGPIOx->AFR[0] |= temp;
		}
		else if((pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber <= 11) && (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber > 7))
		{
			temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << ( 4 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4)));
			pGPIOHandle->pGPIOx->AFR[1] |= temp;
		}
		else if((pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber <= 15) && (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber > 11))
		{
			temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (16 + 4 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4)));
			pGPIOHandle->pGPIOx->AFR[1] |= temp;
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
void GPIO_DeInit(GPIO_regdef_t *pGPIOx)
{
	if(pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}
	else if(pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}
	else if(pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}
	else if(pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	}
	else if(pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	}
	else if(pGPIOx == GPIOF)
	{
		GPIOF_REG_RESET();
	}
	else if(pGPIOx == GPIOG)
	{
		GPIOG_REG_RESET();
	}
	else if(pGPIOx == GPIOH)
	{
		GPIOH_REG_RESET();
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
uint8_t GPIO_ReadIPin(GPIO_regdef_t *pGPIOx, uint8_t Pin_No)
{

	uint8_t temp;

	temp = (uint8_t)( (pGPIOx->IDR >> Pin_No) & 0x00000001);

	return(temp);
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
uint16_t GPIO_ReadIPort(GPIO_regdef_t *pGPIOx)
{

	uint16_t temp;

	temp = (uint16_t)( pGPIOx->IDR & (0x0000000F));

	return(temp);
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
void GPIO_WriteOPin(GPIO_regdef_t *pGPIOx, uint8_t Pin_No, uint8_t value)
{
	if(value == ENABLE)
	{
		//SET THE CORRESPONDING PIN TO HIGH
		pGPIOx->ODR |= (0x1 << Pin_No);
	}
	else
	{
		//SET THE CORRESPONDING PIN TO LOW
		pGPIOx->ODR &= ~(0x1 << Pin_No);
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
void GPIO_WriteOPort(GPIO_regdef_t *pGPIOx, uint16_t value)
{

	pGPIOx->ODR = value;
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
void GPIO_ToggleOPin(GPIO_regdef_t *pGPIOx, uint8_t Pin_No)
{

	pGPIOx->ODR ^= (0x1 << Pin_No);
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
void GPIO_IRQ_ITConfig(uint8_t IRQNumber, uint8_t En_Di)
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

void GPIO_IRQConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{

	uint8_t iprx 		= IRQNumber / 4;
	uint8_t ipr_section = IRQNumber % 4;

	uint8_t shift_amnt  = ((8 * ipr_section) + (8 - NO_PRIORITY_BITS));
	*( NVIC_IPR_BA + (iprx) ) |= (IRQPriority << shift_amnt);


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
void GPIO_IRQHandling(uint8_t Pin_No)
{
	//CLEAR THE IRQ PENDING REGISTER
	if(EXTI->PR & (0x1 << Pin_No))
	{
		//CLEAR THE IRQ PR BY WRITING 1 TO IT
		EXTI->PR |= (0x1 << Pin_No);
	}

}


