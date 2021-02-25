/*
 * stm32f445xx_rcc.h
 *
 *  Created on: Feb 1, 2021
 *      Author: ark
 */


#ifndef INC_STM32F446XX_RCC_H_
#define INC_STM32F446XX_RCC_H_

#include"stm32f446xx.h"

uint32_t RCC_GetPCLK1Value(void);
uint32_t RCC_GetPCLK2Value(void);
uint32_t RCC_GetPLLOutputClock();


#endif /* INC_STM32F446XX_RCC_H_ */
