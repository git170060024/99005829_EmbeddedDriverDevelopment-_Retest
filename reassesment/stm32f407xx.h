
#ifndef INC_STM3F407XX_H_
#define INC_STM3F407XX_H_

#include<stddef.h>
#include<stdint.h>

#define vol volatile


/**********************************START:Processor Specific Details **********************************/
/*
 * ARMS Cortex Mx Processor NVIC ISERx register Addresses
 */

#define NVIC_ISER0          ( (vol uint32_t*)0xE000E100 )
#define NVIC_ISER1          ( (vol uint32_t*)0xE000E104 )
#define NVIC_ISER2          ( (vol uint32_t*)0xE000E108 )
#define NVIC_ISER3          ( (vol uint32_t*)0xE000E10c )


/*
 * ARMS Cortex Mx Processor NVIC ICERx register Addresses
 */
#define NVIC_ICER0 			((vol uint32_t*)0XE000E180)
#define NVIC_ICER1			((vol uint32_t*)0XE000E184)
#define NVIC_ICER2  		((vol uint32_t*)0XE000E188)
#define NVIC_ICER3			((vol uint32_t*)0XE000E18C)


/*
 * ARMS Cortex Mx Processor Priority Register Address Calculation
 */
#define NVIC_PR_BASE_ADDR 	((vol uint32_t*)0xE000E400)

/*
 * ARMS Cortex Mx Processor number of priority bits implemented in Priority Register
 */
#define NO_PR_BITS_IMPLEMENTED  4

/*
 * base addresses of Flash and SRAM memories
 */

#define FLASH_BASEADDR						0x08000000U   		
#define SRAM1_BASEADDR						0x20000000U  		
#define SRAM2_BASEADDR						0x2001C000U 		
#define ROM_BASEADDR					   	0x1FFF0000U
#define SRAM 								      SRAM1_BASEADDR


/*
 * ABHx and ABPx Bus Peripheral base addresses
 */

#define PERIPH_BASEADDR 						0x40000000U
#define ABP1PERIPH_BASEADDR						PERIPH_BASEADDR
#define ABP2PERIPH_BASEADDR						0x40010000U
#define ABH1PERIPH_BASEADDR						0x40020000U
#define ABH2PERIPH_BASEADDR						0x50000000U

/*
 * Base addresses of peripherals which are hanging on ABH1 bus
 */

#define GPIOA_BASEADDR                   (ABH1PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR                   (ABH1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR 					 (ABH1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR 					 (ABH1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR 					 (ABH1PERIPH_BASEADDR + 0x1000)
#define GPIOF_BASEADDR 					 (ABH1PERIPH_BASEADDR + 0x1400)
#define GPIOG_BASEADDR 					 (ABH1PERIPH_BASEADDR + 0x1800)
#define GPIOH_BASEADDR 					 (ABH1PERIPH_BASEADDR + 0x1C00)
#define GPIOI_BASEADDR 					 (ABH1PERIPH_BASEADDR + 0x2000)
#define RCC_BASEADDR                     (ABH1PERIPH_BASEADDR + 0x3800)
/*
 * Base addresses of peripherals which are hanging on APB1 bus
 */
#define I2C1_BASEADDR					  	(ABP1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR					  	(ABP1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR					   	(ABP1PERIPH_BASEADDR + 0x5C00)

#define SPI2_BASEADDR					  	(ABP1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR					  	(APB1PERIPH_BASEADDR + 0x3C00)

#define USART2_BASEADDR						(ABP1PERIPH_BASEADDR + 0x4400)
#define USART3_BASEADDR						(ABP1PERIPH_BASEADDR + 0x4800)
#define UART4_BASEADDR						(ABP1PERIPH_BASEADDR + 0x4C00)
#define UART5_BASEADDR						(ABP1PERIPH_BASEADDR + 0x5000)

/*
 * Base addresses of peripherals which are hanging on APB2 bus
 */
#define EXTI_BASEADDR					  	(ABP2PERIPH_BASEADDR + 0x3C00)
#define SPI1_BASEADDR					    (ABP2PERIPH_BASEADDR + 0x3000)
#define SYSCFG_BASEADDR        		(APB2PERIPH_BASEADDR + 0x3800)
#define USART1_BASEADDR						(ABP2PERIPH_BASEADDR + 0x1000)
#define USART6_BASEADDR						(APB2PERIPH_BASEADDR + 0x1400)

/*
 * Compared to number of registers of SPI peripheral of STM32Lx or STM32F0x family of MCUs
 */

typedef struct
{
	vol uint32_t MODER;                       
	vol uint32_t OTYPER;                      
	vol uint32_t OSPEEDR;
	vol uint32_t PUPDR;
	vol uint32_t IDR;
	vol uint32_t ODR;
	vol uint32_t BSRR;
	vol uint32_t LCKR;
	vol uint32_t AFR[2];					
}GPIO_RegDef_t;



/*
 * peripheral register definition structure for RCC
 */
typedef struct
{
  vol uint32_t CR;            /*Address offset: 0x00 */
  vol uint32_t PLLCFGR;       /*Address offset: 0x04 */
  vol uint32_t CFGR;          /*Address offset: 0x08 */
  vol uint32_t CIR;           /*Address offset: 0x0C */
  vol uint32_t AHB1RSTR;      /*Address offset: 0x10 */
  vol uint32_t AHB2RSTR;      /*Address offset: 0x14 */
  vol uint32_t AHB3RSTR;      /*Address offset: 0x18 */
  uint32_t      RESERVED0;     /*Reserved, 0x1C*/
  vol uint32_t APB1RSTR;      /*Address offset: 0x20 */
  vol uint32_t APB2RSTR;      /*Address offset: 0x24 */
  uint32_t      RESERVED1[2];  /*!< Reserved, 0x28-0x2C */
  vol uint32_t AHB1ENR;       /*Address offset: 0x30 */
  vol uint32_t AHB2ENR;       /*Address offset: 0x34 */
  vol uint32_t AHB3ENR;       /*Address offset: 0x38 */
  uint32_t      RESERVED2;     /*Reserved, 0x3C*/
  vol uint32_t APB1ENR;       /*Address offset: 0x40 */
  vol uint32_t APB2ENR;       /*Address offset: 0x44 */
  uint32_t      RESERVED3[2];  /*Reserved, 0x48-0x4C   */
  vol uint32_t AHB1LPENR;     /*Address offset: 0x50 */
  vol uint32_t AHB2LPENR;     /*Address offset: 0x54 */
  vol uint32_t AHB3LPENR;     /*Address offset: 0x58 */
  uint32_t      RESERVED4;     /*Reserved, 0x5C*/
  vol uint32_t APB1LPENR;     /*Address offset: 0x60 */
  vol uint32_t APB2LPENR;     /*Address offset: 0x64 */
  uint32_t      RESERVED5[2];  /*Reserved, 0x68-0x6C  */
  vol uint32_t BDCR;          /*Address offset: 0x70 */
  vol uint32_t CSR;           /*Address offset: 0x74 */
  uint32_t      RESERVED6[2];  /*Reserved, 0x78-0x7C  */
  vol uint32_t SSCGR;         /*Address offset: 0x80 */
  vol uint32_t PLLI2SCFGR;    /*Address offset: 0x84 */
  vol uint32_t PLLSAICFGR;    /*Address offset: 0x88 */
  vol uint32_t DCKCFGR;       /*Address offset: 0x8C */
  vol uint32_t CKGATENR;      /*Address offset: 0x90 */
  vol uint32_t DCKCFGR2;      /*Address offset: 0x94 */

} RCC_RegDef_t;



/*
 * peripheral register definition structure for EXTI
 */
typedef struct
{
	vol uint32_t IMR;    /*!< Give a short description,          	  	    Address offset: 0x00 */
	vol uint32_t EMR;    /*               						Address offset: 0x04 */
	vol uint32_t RTSR;   /* 									     Address offset: 0x08 */
	vol uint32_t FTSR;   /*										Address offset: 0x0C */
	vol uint32_t SWIER;  /* 									   Address offset: 0x10 */
	vol uint32_t PR;     /*                  					   Address offset: 0x14 */

}EXTI_RegDef_t;


/*
 * peripheral register definition structure for SYSCFG
 */
typedef struct
{
	vol uint32_t MEMRMP;       /*Address offset: 0x00      */
	vol uint32_t PMC;          /*  Address offset: 0x04      */
	vol uint32_t EXTICR[4];    /*Address offset: 0x08-0x14 */
	uint32_t      RESERVED1[2];  /*  Reserved, 0x18-0x1C    	*/
	vol uint32_t CMPCR;        /* Address offset: 0x20      */
	uint32_t      RESERVED2[2];  /* Reserved, 0x24-0x28 	    */
	vol uint32_t CFGR;         /* Address offset: 0x2C   	*/
} SYSCFG_RegDef_t;



/*
 * peripheral definitions ( Peripheral base addresses typecasted to xxx_RegDef_t)
 */

#define GPIOA  				((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB  				((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC  				((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD  				((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE  				((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF  				((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG  				((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH  				((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI  				((GPIO_RegDef_t*)GPIOI_BASEADDR)

#define RCC 				((RCC_RegDef_t*)RCC_BASEADDR)
#define EXTI				((EXTI_RegDef_t*)EXTI_BASEADDR)
#define SYSCFG				((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

/*
 * Clock Enable Macros for GPIOx peripherals
 */

#define GPIOA_PCLK_EN()    	(RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()		(RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()		(RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()		(RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()		(RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN()		(RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN()		(RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN()		(RCC->AHB1ENR |= (1 << 7))
#define GPIOI_PCLK_EN()		(RCC->AHB1ENR |= (1 << 8))




/*
 * Clock Enable Macros for SYSCFG peripheral
 */
#define SYSCFG_PCLK_EN() (RCC->APB2ENR |= (1 << 14))


/*
 * Clock Disable Macros for GPIOx peripherals
 */
#define GPIOA_PCLK_DI() 	(RCC->AHB1ENR &= ~(1<<0)) // reset
#define GPIOB_PCLK_DI() 	(RCC->AHB1ENR &= ~(1<<1))
#define GPIOC_PCLK_DI() 	(RCC->AHB1ENR &= ~(1<<2))
#define GPIOD_PCLK_DI() 	(RCC->AHB1ENR &= ~(1<<3))
#define GPIOE_PCLK_DI() 	(RCC->AHB1ENR &= ~(1<<4))
#define GPIOF_PCLK_DI() 	(RCC->AHB1ENR &= ~(1<<5))
#define GPIOG_PCLK_DI() 	(RCC->AHB1ENR &= ~(1<<6))
#define GPIOH_PCLK_DI() 	(RCC->AHB1ENR &= ~(1<<7))
#define GPIOI_PCLK_DI() 	(RCC->AHB1ENR &= ~(1<<8))



/*
 *  Macros to reset GPIOx peripherals
 */
#define GPIOA_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0)); }while(0)
#define GPIOB_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1)); }while(0)
#define GPIOC_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2)); }while(0)
#define GPIOD_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3)); }while(0)
#define GPIOE_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4)); }while(0)
#define GPIOF_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &= ~(1 << 5)); }while(0)
#define GPIOG_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &= ~(1 << 6)); }while(0)
#define GPIOH_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7)); }while(0)
#define GPIOI_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 8)); (RCC->AHB1RSTR &= ~(1 << 8)); }while(0)


/*
 *  returns port code for given GPIOx base address
 */
/*
 * This macro returns a code( between 0 to 7) for a given GPIO base address(x)
 */
#define GPIO_BASEADDR_TO_CODE(x)      ( (x == GPIOA)?0:\
										(x == GPIOB)?1:\
										(x == GPIOC)?2:\
										(x == GPIOD)?3:\
								        (x == GPIOE)?4:\
								        (x == GPIOF)?5:\
								        (x == GPIOG)?6:\
								        (x == GPIOH)?7: \
								        (x == GPIOI)?8:0)


/*
 * IRQ(Interrupt Request) Numbers of STM32F407x MCU
 */

#define IRQ_NO_EXTI0 		6
#define IRQ_NO_EXTI1 		7
#define IRQ_NO_EXTI2 		8
#define IRQ_NO_EXTI3 		9
#define IRQ_NO_EXTI4 		10
#define IRQ_NO_EXTI9_5 		23
#define IRQ_NO_EXTI15_10 	40
#define IRQ_NO_SPI1			35
#define IRQ_NO_SPI2         36
#define IRQ_NO_SPI3         51
#define IRQ_NO_SPI4
#define IRQ_NO_I2C1_EV      31
#define IRQ_NO_I2C1_ER      32
#define IRQ_NO_USART1	    37
#define IRQ_NO_USART2	    38
#define IRQ_NO_USART3	    39
#define IRQ_NO_UART4	    52
#define IRQ_NO_UART5	    53
#define IRQ_NO_USART6	    71


/*
 * macros for all the possible priority levels
 */
#define NVIC_IRQ_PRI0      0
#define NVIC_IRQ_PRI15    15


//some generic macros

#define ENABLE 				1
#define DISABLE 			0
#define SET 				ENABLE
#define RESET 				DISABLE
#define GPIO_PIN_SET        SET
#define GPIO_PIN_RESET      RESET
#define FLAG_RESET          RESET
#define FLAG_SET 			SET



#include "stm32f407xx_gpio_driver.h"


#endif /* INC_STM3F407XX_H_ */
