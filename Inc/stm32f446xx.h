/*
 * stm32f446xx.h
 *
 *  Created on: Feb 1, 2021
 *      Author: ark
 */

#ifndef INC_STM32F446XX_H_
#define INC_STM32F446XX_H_

/*
 * THE FOLLOWING MACROS DEFINE BASE ADDRESSES OF SEVERAL PERIPHERALS AS WELL AS MEMORY AND SYSTEM REGIONS
 * NOTE: ALL ARE BASE ADDRESSES
 * _BA EXPLICITLY DEFINES BASE ADDRESS
 */

#include<stdint.h>
#include<stddef.h>
/*************************************** PROCESSOR SPECIFIC DETAILS ***************************************************/
/*
 * ARM CORTEX Mx NVIC ISERx REGISTER ADDRESSES
 */
#define NVIC_ISER0			((__vo uint32_t*)0xE000E100)
#define NVIC_ISER1			((__vo uint32_t*)0xE000E104)
#define NVIC_ISER2			((__vo uint32_t*)0xE000E108)
#define NVIC_ISER3			((__vo uint32_t*)0xE000E10C)
#define NVIC_ISER4			((__vo uint32_t*)0xE000E110)
#define NVIC_ISER5			((__vo uint32_t*)0xE000E114)
#define NVIC_ISER6			((__vo uint32_t*)0xE000E118)
#define NVIC_ISER7			((__vo uint32_t*)0xE000E11C)

/*
 * ARM CORTEX Mx NVIC ICERx REGISTER ADDRESSES
 */
#define NVIC_ICER0			((__vo uint32_t*)0XE000E180)
#define NVIC_ICER1			((__vo uint32_t*)0xE000E184)
#define NVIC_ICER2			((__vo uint32_t*)0xE000E188)
#define NVIC_ICER3			((__vo uint32_t*)0xE000E18C)
#define NVIC_ICER4			((__vo uint32_t*)0xE000E190)
#define NVIC_ICER5			((__vo uint32_t*)0xE000E194)
#define NVIC_ICER6			((__vo uint32_t*)0xE000E198)
#define NVIC_ICER7			((__vo uint32_t*)0xE000E19C)

/*
 * ARM CORTEX Mx IPR PRIORITY REGISTER ADDRESSES
 */
#define NVIC_IPR_BA			((__vo uint32_t*)0xE000E400)

#define NO_PRIORITY_BITS	4

/************************************************** END ***********************************************************************/
/*
 * MACRO TO DEFINE MEMORY REGION
 */
#define	FLASH_BA		0X08000000U
#define	SRAM_1			0X20000000U
#define	SRAM_2			SRAM_1 + (112U * 1024U)
#define	SRAM			SRAM_1
#define	ROM_BA			0x1FFF0000U
#define	ROM				ROM_BA
/*
 * MACRO TO DEFINE BUS PERIPHERALS
 */
#define	AHB1_BA			0X40020000U
#define	AHB2_BA 		0X50000000U
#define AHB3_BA			0XA0000000U

#define	APB1_BA			0X40000000U
#define	APB2_BA			0X40010000U
/*
 * MACRO TO DEFINE GPIO PERIPHERALS
 */
#define	GPIOA_BA		0x40020000U
#define	GPIOB_BA		0x40020400U
#define GPIOC_BA		0x40020800U
#define GPIOD_BA		0x40020C00U
#define GPIOE_BA		0x40021000U
#define GPIOF_BA		0x40021400U
#define GPIOG_BA		0x40021800U
#define GPIOH_BA		0x40021C00U

#define RCC_BA			0x40023800U
/*
 * MACRO TO DEFINE I2C PERIPHERALS
 */
#define 	I2C1_BA			0x40005400U
#define	I2C2_BA			0x40005800U
#define	I2C3_BA			0x40005C00U
/*
 * MACRO TO DEFINE SPI PERIPHERALS
 */
#define SPI1_BA			0x40013000U
#define	SPI2_BA			0x40003800U
#define	SPI3_BA			0x40003C00U
#define SPI4_BA			0x40013400U
/*
 * MACRO TO DEFINE USART PERIPHERAL
 */
#define	USART1_BA			0x40011000U
#define	USART2_BA			0x40004400U
#define	USART3_BA			0x40004800U
#define	UART4_BA			0x40004C00U
#define	UART5_BA			0x40005000U
#define	USART6_BA			0x40011400U
/*
 * DEFINES SYSTEM PERIPHERALS
 */
#define	EXTI_BA				0x40013C00U
#define	SYSCFG_BA			0x40013800U

/************************************************************** MISC MACROS **********************************************************************************/

#define __vo						volatile
#define __weak					__attribute__((weak))

#define SET							1
#define RESET						0
#define ENABLE  				1
#define DISABLE 				0
#define GPIO_PIN_SET		1
#define GPIO_PIN_RESET	0
#define FLAG_SET				1
#define FLAG_RESET			0


/***************************************************************** PERIPHERAL DEFINITION REGISTERS **********************************************************/
/*
 * THE FOLLOWING STRUCTURES DEFINE VARIOUS PERIPHERAL RELATED REGISTERS
 */

// STRUCTURE TO INITIALIZE GPIO PERIPHERALS AND MACROS TO EXPOSE GPIO REGs

typedef struct{

	__vo uint32_t MODER;
	__vo uint32_t OTYPER;
	__vo uint32_t OSPEEDR;
	__vo uint32_t PUPDR;
	__vo uint32_t IDR;
	__vo uint32_t ODR;
	__vo uint32_t BSRR;
	__vo uint32_t LCKR;
	__vo uint32_t AFR[2];

}GPIO_regdef_t;

//STRUCTURE TO INITIALIZE RCC PERIPHERALS AND MACROS TO EXPOSE RCC REGs

typedef struct{

	__vo uint32_t IMR;
	__vo uint32_t EMR;
	__vo uint32_t RTSR;
	__vo uint32_t FTSR;
	__vo uint32_t SWIER;
	__vo uint32_t PR;

}EXTI_RegDef_t;

typedef struct{

	/*
	 * THIS STRUCTS CONTAINS SYSCFG REGS
	 */

	__vo uint32_t MEMRM;
	__vo uint32_t PMC;
	__vo uint32_t EXTICR[4];
	__vo uint32_t CMPCR;
	__vo uint32_t CFGR;

}SYSCFG_RegDef_t;

typedef struct{

	/*
	 * THIS STRUCT CONTAINS AHB && APB CONFIG REGISTERS AS WELL AS
	 * AHBLP && APBLP CONFIG REGS
	 */

	__vo uint32_t CR;
	__vo uint32_t PLLCFGR;
	__vo uint32_t CFGR;
	__vo uint32_t CIR;
	__vo uint32_t AHB1RSTR;
	__vo uint32_t AHB2RSTR;
	__vo uint32_t AHB3RSTR;
	__vo uint32_t RESERVED_1;

	__vo uint32_t APB1RSTR;
	__vo uint32_t APB2RSTR;
	__vo uint32_t RESERVED_2[2];

	__vo uint32_t AHB1ENR;
	__vo uint32_t AHB2ENR;
	__vo uint32_t AHB3ENR;
	__vo uint32_t RESERVED_3;

	__vo uint32_t APB1ENR;
	__vo uint32_t APB2ENR;
	__vo uint32_t RESERVED_4[2];

	__vo uint32_t AHB1LPENR;
	__vo uint32_t AHB2LPENR;
	__vo uint32_t AHB3LPENR;
	__vo uint32_t RESERVED_5;

	__vo uint32_t APB1LPENR;
	__vo uint32_t APB2LPENR;
	__vo uint32_t RESERVED_6[2];

	__vo uint32_t BDCR;
	__vo uint32_t CSR;
	__vo uint32_t RESERVED_7[2];

	__vo uint32_t SSCGR;
	__vo uint32_t PLLI2SCFGR;
	__vo uint32_t PLLSAICFGR;
	__vo uint32_t DCKCFGR;
	__vo uint32_t CKGATRSTR;
	__vo uint32_t DCKCFGR2;

}RCC_RegDef_t;

typedef struct{

	/*
	 * THIS STRUCTURE CONTAINS DEFINTTIONS OF SPI AND I2S
	 */

	//SPI
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t SR;
	__vo uint32_t DR;
	__vo uint32_t CRCPR;
	__vo uint32_t RXCRCR;
	__vo uint32_t TXCRCR;

	//I2S
	__vo uint32_t CFGR;
	__vo uint32_t PR;

}SPI_RegDef_t;


typedef struct{

	/*
	 * THIS 	CONTAINS 	DEFINITION 	OF 	I2C		PERIPHERAL
	 */

		__vo uint32_t CR1;
		__vo uint32_t CR2;
		__vo uint32_t OAR1;
		__vo uint32_t OAR2;
		__vo uint32_t DR;
		__vo uint32_t SR1;
		__vo uint32_t SR2;
		__vo uint32_t CCR;
		__vo uint32_t TRISE;
		__vo uint32_t FLTR;

}I2C_RegDef_t;


typedef struct{

	/*
	* THIS 	STRUCTURE 		CONTAINS 		DEFINTTIONS 		OF 		USART
	*/

	__vo uint32_t	SR;
	__vo uint32_t	DR;
	__vo uint32_t	BRR;
	__vo uint32_t	CR1;
	__vo uint32_t	CR2;
	__vo uint32_t	CR3;
	__vo uint32_t	GTPR;


}USART_RegDef_t;
/************************************************************************************************************************************

  	  	  	  	  	  	  	  	  	  	  	  	 PERIPHERAL SPECIFIC MACRO SECTION

*************************************************************************************************************************************/


#define GPIOA			(GPIO_regdef_t *)(GPIOA_BA)
#define GPIOB			(GPIO_regdef_t *)(GPIOB_BA)
#define GPIOC			(GPIO_regdef_t *)(GPIOC_BA)
#define GPIOD			(GPIO_regdef_t *)(GPIOD_BA)
#define GPIOE			(GPIO_regdef_t *)(GPIOE_BA)
#define GPIOF				(GPIO_regdef_t *)(GPIOF_BA)
#define GPIOG			(GPIO_regdef_t *)(GPIOG_BA)
#define GPIOH			(GPIO_regdef_t *)(GPIOH_BA)



#define RCC 				((RCC_RegDef_t *)(RCC_BA))
#define EXTI				((EXTI_RegDef_t *)(EXTI_BA))
#define SYSCFG			((SYSCFG_RegDef_t *)(SYSCFG_BA))


#define SPI1				((SPI_RegDef_t *)(SPI1_BA))
#define SPI2				((SPI_RegDef_t *)(SPI2_BA))
#define SPI3				((SPI_RegDef_t *)(SPI3_BA))
#define SPI4				((SPI_RegDef_t *)(SPI4_BA))


#define USART1			((USART_RegDef_t *)(USART1_BA))
#define USART2			((USART_RegDef_t *)(USART2_BA))
#define USART3			((USART_RegDef_t *)(USART3_BA))
#define UART4			((USART_RegDef_t *)(UART4_BA))
#define UART5			((USART_RegDef_t *)(UART5_BA))
#define USART6			((USART_RegDef_t *)(USART6_BA))


#define I2C1				((I2C_RegDef_t *)(I2C1_BA))
#define I2C2				((I2C_RegDef_t *)(I2C2_BA))
#define I2C3				((I2C_RegDef_t *)(I2C3_BA))


/********************************** ENABLE MACROS *************************************/
/***********
 *
 * GPIO CLOCK ENABLE MACROS
 ***********/
#define GPIOA_CLK_EN()			(RCC->AHB1ENR |= (0x1 << 0))
#define GPIOB_CLK_EN()			(RCC->AHB1ENR |= (0x1 << 1))
#define GPIOC_CLK_EN()			(RCC->AHB1ENR |= (0x1 << 2))
#define GPIOD_CLK_EN()			(RCC->AHB1ENR |= (0x1 << 3))
#define GPIOE_CLK_EN()			(RCC->AHB1ENR |= (0x1 << 4))
#define GPIOF_CLK_EN()			(RCC->AHB1ENR |= (0x1 << 5))
#define GPIOG_CLK_EN()			(RCC->AHB1ENR |= (0x1 << 6))
#define GPIOH_CLK_EN()			(RCC->AHB1ENR |= (0x1 << 7))



/***********
 *
 * I2C CLOCK ENABLE MACROS
 ***********/
#define I2C1_CLK_EN()			(RCC->APB1ENR |= (0X1 << 21))
#define I2C2_CLK_EN()			(RCC->APB1ENR |= (0X1 << 22))
#define I2C3_CLK_EN()			(RCC->APB1ENR |= (0X1 << 23))



/***********
 *
 * SPI CLOCK ENABLE MACROS
 ***********/
#define SPI1_CLK_EN()			(RCC->APB2ENR |= (0X1 << 12))
#define SPI2_CLK_EN()			(RCC->APB1ENR |= (0X1 << 14))
#define SPI3_CLK_EN()			(RCC->APB1ENR |= (0X1 << 15))
#define SPI4_CLK_EN()			(RCC->APB2ENR |= (0X1 << 13))



/***********
 *
 * USART/UART CLOCK ENABLE MACROS
 ***********/
#define USART1_CLK_EN()			(RCC->APB2ENR |= (0X1 << 4))
#define USART2_CLK_EN()			(RCC->APB1ENR |= (0X1 << 17))
#define USART3_CLK_EN()			(RCC->APB1ENR |= (0X1 << 18))
#define UART4_CLK_EN()			(RCC->APB1ENR |= (0X1 << 19))
#define UART5_CLK_EN()			(RCC->APB1ENR |= (0X1 << 20))
#define USART6_CLK_EN()			(RCC->APB2ENR |= (0X1 << 5))


/***********
 *
 * SYSCFG CLOCK ENABLE MACROS
 ***********/
#define	SYSCFG_CLK_EN()			(RCC->APB2ENR |= (0x1 << 14))

/***************************** DISABLE MACROS ********************************/
/***********
 *
 * GPIO CLOCK DISABLE MACROS
 ***********/
#define GPIOA_CLK_DI()			(RCC->AHB1RSTR |= (0x1 << 0))
#define GPIOB_CLK_DI()			(RCC->AHB1RSTR |= (0x1 << 1))
#define GPIOC_CLK_DI()				(RCC->AHB1RSTR |= (0x1 << 2))
#define GPIOD_CLK_DI()			(RCC->AHB1RSTR |= (0x1 << 3))
#define GPIOE_CLK_DI()				(RCC->AHB1RSTR |= (0x1 << 4))
#define GPIOF_CLK_DI()				(RCC->AHB1RSTR |= (0x1 << 5))
#define GPIOG_CLK_DI()			(RCC->AHB1RSTR |= (0x1 << 6))
#define GPIOH_CLK_DI()			(RCC->AHB1RSTR |= (0x1 << 7))



/***********
 *
 * I2C CLOCK DISABLE MACROS
 ***********/
#define I2C1_CLK_DI()					(RCC->APB1RSTR |= (0X1 << 21))
#define I2C2_CLK_DI()				(RCC->APB1RSTR |= (0X1 << 22))
#define I2C3_CLK_DI()				(RCC->APB1RSTR |= (0X1 << 23))



/***********
 *
 * SPI CLOCK DISABLE MACROS
 ***********/
#define SPI1_CLK_DI()				(RCC->APB2RSTR |= (0X1 << 12))
#define SPI2_CLK_DI()				(RCC->APB1RSTR |= (0X1 << 14))
#define SPI3_CLK_DI()				(RCC->APB1RSTR |= (0X1 << 15))
#define SPI4_CLK_DI()				(RCC->APB2RSTR |= (0X1 << 13))



/***********
 *
 * USART/UART CLOCK DISABLE MACROS
 ***********/
#define USART1_CLK_DI()			(RCC->APB2RSTR |= (0X1 << 4))
#define USART2_CLK_DI()			(RCC->APB1RSTR |= (0X1 << 17))
#define USART3_CLK_DI()			(RCC->APB1RSTR |= (0X1 << 18))
#define UART4_CLK_DI()			(RCC->APB1RSTR |= (0X1 << 19))
#define UART5_CLK_DI()				(RCC->APB1RSTR |= (0X1 << 20))
#define USART6_CLK_DI()			(RCC->APB2RSTR |= (0X1 << 5))


/***********
 *
 * SYSCFG CLOCK DISABLE MACROS
 ***********/
#define	SYSCFG_CLK_DI()			(RCC->APB2RSTR |= (0x1 << 14))



/***************************** RESET MACROS ********************************/
/***********
 *
 * GPIO CLOCK RESET MACROS
 ***********/
#define GPIOA_REG_RESET()			do{ (RCC->AHB1RSTR |= (0x1 << 0)); (RCC->AHB1RSTR &= ~(0x1 << 0)); }while(0)
#define GPIOB_REG_RESET()			do{ (RCC->AHB1RSTR |= (0x1 << 1)); (RCC->AHB1RSTR &= ~(0x1 << 0)); }while(0)
#define GPIOC_REG_RESET()			do{ (RCC->AHB1RSTR |= (0x1 << 2)); (RCC->AHB1RSTR &= ~(0x1 << 0)); }while(0)
#define GPIOD_REG_RESET()			do{ (RCC->AHB1RSTR |= (0x1 << 3)); (RCC->AHB1RSTR &= ~(0x1 << 0)); }while(0)
#define GPIOE_REG_RESET()			do{ (RCC->AHB1RSTR |= (0x1 << 4)); (RCC->AHB1RSTR &= ~(0x1 << 0)); }while(0)
#define GPIOF_REG_RESET()			do{ (RCC->AHB1RSTR |= (0x1 << 5)); (RCC->AHB1RSTR &= ~(0x1 << 0)); }while(0)
#define GPIOG_REG_RESET()			do{ (RCC->AHB1RSTR |= (0x1 << 6)); (RCC->AHB1RSTR &= ~(0x1 << 0)); }while(0)
#define GPIOH_REG_RESET()			do{ (RCC->AHB1RSTR |= (0x1 << 7)); (RCC->AHB1RSTR &= ~(0x1 << 0)); }while(0)



/***********
 *
 * I2C CLOCK RESET MACROS
 ***********/
#define I2C1_REG_RESET()				do{ (RCC->APB1RSTR |= (0x1 << 21)); (RCC->APB1RSTR &= ~(0x1 << 21)); }while(0);
#define I2C2_REG_RESET()				do{ (RCC->APB1RSTR |= (0x1 << 22)); (RCC->APB1RSTR &= ~(0x1 << 22)); }while(0);
#define I2C3_REG_RESET()				do{ (RCC->APB1RSTR |= (0x1 << 23)); (RCC->APB1RSTR &= ~(0x1 << 23)); }while(0);



/***********
 *
 * SPI CLOCK RESET MACROS
 ***********/
#define SPI1_REG_RESET()				do{ (RCC->APB2RSTR |= (0x1 << 12)); (RCC->APB2RSTR &= ~(0x1 << 12)); }while(0);
#define SPI2_REG_RESET()			do{ (RCC->APB1RSTR |= (0x1 << 14)); (RCC->APB2RSTR &= ~(0x1 << 12)); }while(0);
#define SPI3_REG_RESET()			do{ (RCC->APB1RSTR |= (0x1 << 15)); (RCC->APB2RSTR &= ~(0x1 << 12)); }while(0);
#define SPI4_REG_RESET()			do{ (RCC->APB2RSTR |= (0x1 << 13)); (RCC->APB2RSTR &= ~(0x1 << 12)); }while(0);



/***********
 *
 * USART/UART CLOCK RESET MACROS
 ***********/
#define USART1_REG_RESET()		do{ (RCC->APB2RSTR |= (0x1 << 4)) ; ((RCC->APB2RSTR &= ~(0x1 << 4)); }while(0);
#define USART2_REG_RESET()		do{ (RCC->APB1RSTR |= (0x1 << 17)); ((RCC->APB2RSTR &= ~(0x1 << 4)); }while(0);
#define USART3_REG_RESET()		do{ (RCC->APB1RSTR |= (0x1 << 18)); ((RCC->APB2RSTR &= ~(0x1 << 4)); }while(0);
#define UART4_REG_RESET()	  		do{	(RCC->APB1RSTR |= (0x1 << 19)); ((RCC->APB2RSTR &= ~(0x1 << 4)); }while(0);
#define UART5_REG_RESET()			do{	(RCC->APB1RSTR |= (0x1 << 20)); ((RCC->APB2RSTR &= ~(0x1 << 4)); }while(0);
#define USART6_REG_RESET()		do{	(RCC->APB2RSTR |= (0x1 << 5)) ; ((RCC->APB2RSTR &= ~(0x1 << 4)); }while(0);


/***********
 *
 * SYSCFG CLOCK RESET MACROS
 ***********/
#define	SYSCFG_REG_RESET()		do{ (RCC->APB2RSTR |= (0x1 << 14)); (RCC->APB2RSTR &= ~(0x1 << 14)); }while(0);


#define GPIO_BA_CODE(x)			((x == GPIOA) ? 0:\
								(x == GPIOB) ? 1:\
								(x == GPIOC) ? 2:\
								(x == GPIOD) ? 3:\
								(x == GPIOE) ? 4:\
								(x == GPIOF) ? 5:\
								(x == GPIOG) ? 6:\
								(x == GPIOH) ? 7: -1 )
/*
 * INTERRUPT REQUEST NUMBER (IRQ NUMBERS) OF THE MICROCONTROLLER
 */

#define IRQ_NO_EXTI0			6
#define IRQ_NO_EXTI1			7
#define IRQ_NO_EXTI2			8
#define IRQ_NO_EXTI3			9
#define IRQ_NO_EXTI4			10
#define IRQ_NO_EXTI9_5		23
#define IRQ_NO_EXTI15_10	40


#define IRQ_NO_SPI1				35
#define IRQ_NO_SPI2				36
#define IRQ_NO_SPI3				51
#define IRQ_NO_SPI4				84


#define IRQ_NO_I2C1_EV		31
#define IRQ_NO_I2C1_ER		32
#define IRQ_NO_I2C2_EV		33
#define IRQ_NO_I2C2_ER		34
#define IRQ_NO_I2C3_EV		72
#define IRQ_NO_I2C3_ER		73


/************************************************************************************************************************************
 * 																		 MACROS FOR SPI BIT DEFINITIONS														   							*
 ************************************************************************************************************************************/


#define SPI_CR1_CPHA				0
#define SPI_CR1_CPOL				1
#define SPI_CR1_MSTR				2
#define SPI_CR1_BR					3
#define SPI_CR1_SPE					6
#define SPI_CR1_LSB					7
#define SPI_CR1_SSI					8
#define SPI_CR1_SSM					9
#define SPI_CR1_RXONLY			10
#define SPI_CR1_DFF					11
#define SPI_CR1_CRCNXT			12
#define SPI_CR1_CRCEN				13
#define SPI_CR1_BIDIOE			14
#define SPI_CR1_BIDIMODE		15



#define SPI_CR2_RXDMAEN		0
#define SPI_CR2_TXDMAEN		1
#define SPI_CR2_SSOE				2
#define SPI_CR2_FRF					4
#define SPI_CR2_ERRIE				5
#define SPI_CR2_RXNEIE			6
#define SPI_CR2_TXEIE				7



#define SPI_SR_RXNE					0
#define SPI_SR_TXE					1
#define SPI_SR_CHSIDE				2
#define SPI_SR_UDR					3
#define SPI_SR_CRCERR				4
#define SPI_SR_MODF				5
#define SPI_SR_OVR					6
#define SPI_SR_BSY					7
#define SPI_SR_FRE					8



/************************************************************************************************************************************
 *					 												 MACROS FOR I2C BIT DEFINITIONS																						*
 ************************************************************************************************************************************/



#define I2C_CR1_PE					0
#define I2C_CR1_SMBUS				1
#define I2C_CR1_SMBTYPE			3
#define I2C_CR1_ENARP				4
#define I2C_CR1_ENPEC				5
#define I2C_CR1_ENGC				6
#define I2C_CR1_NOSTRETCH	7
#define I2C_CR1_START				8
#define I2C_CR1_STOP				9
#define I2C_CR1_ACK					10
#define I2C_CR1_POS					11
#define I2C_CR1_PEC					12
#define I2C_CR1_ALERT				13
#define I2C_CR1_SWRST				15



#define I2C_CR2_FREQ				0
#define I2C_CR2_ITERREN			8
#define I2C_CR2_ITEVTEN			9
#define I2C_CR2_ITBUFEN			10
#define I2C_CR2_DMAEN			11
#define I2C_CR2_LAST				12



#define I2C_SR1_SB					0
#define I2C_SR1_ADDR				1
#define I2C_SR1_BTF					2
#define I2C_SR1_ADD10				3
#define I2C_SR1_STOPF				4
#define I2C_SR1_RxNE				6
#define I2C_SR1_TxE				    7
#define I2C_SR1_BERR				8
#define I2C_SR1_ARLO				9
#define I2C_SR1_AF					10
#define I2C_SR1_OVR					11
#define I2C_SR1_PECERR			12
#define I2C_SR1_TIMEOUT			14
#define I2C_SR1_SMBALERT		15



#define I2C_SR2_MSL					0
#define I2C_SR2_BSY					1
#define I2C_SR2_TRA					2
#define I2C_SR2_GENCALL			4
#define I2C_SR2_SMBDEFAULT	5
#define I2C_SR2_SMBHOST		6
#define I2C_SR2_DUALF				7
#define I2C_SR2_PEC					8



#define I2C_CCR_CCR					0
#define I2C_CCR_DUTY				14
#define I2C_CCR_FS					15


/*
#include"stm32f446xx_gpio_cpp.h"
#include"stm32f446xx_spi.h"
#include"stm32f446xx_rcc.h"
#include"stm32f446xx_usart.h"
#include"stm32f446xx_i2c.h"
*/


#endif /* INC_STM32F446XX_H_ */
