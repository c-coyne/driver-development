
/*=====================================================================================*\
| Author:   Christopher Coyne                                          July 25th, 2024  |
| --------------------------------------------------------------------------------------|
| Date:     July 25th, 2024                                                             |
| --------------------------------------------------------------------------------------|
| MODULE:     [ stm32f407xx ]                                                           |
| FILE:       stm32f407xx.h                                                             |
| --------------------------------------------------------------------------------------|
| DESCRIPTION:                                                                          |
|    This file defines the base addresses for various memory regions and peripheral     |
|    registers for the STM32F407 microcontroller, providing essential mappings for the  |
|    driver and application layers to access and control hardware components            |
|    efficiently.                                                                       |
\*=====================================================================================*/

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

/****************************************************
 *  Include files                                   *
 ****************************************************/

#include <stdint.h>

/****************************************************
 *  Generic macros                                  *
 ****************************************************/

#define ENABLE                  1
#define DISABLE                 0
#define SET                     ENABLE
#define RESET                   DISABLE
#define GPIO_PIN_SET            SET
#define GPIO_PIN_RESET          RESET
#define FLAG_SET                SET
#define FLAG_RESET              RESET

/****************************************************
 *  Processor specific details                      *
 ****************************************************/

// ARM Cortex-M4 processor NVIC ISERx register adddresses
#define NVIC_ISER0              ( (volatile uint32_t*)0xE000E100 )
#define NVIC_ISER1              ( (volatile uint32_t*)0xE000E104 )
#define NVIC_ISER2              ( (volatile uint32_t*)0xE000E108 )
#define NVIC_ISER3              ( (volatile uint32_t*)0xE000E10C )
// ARM Cortex-M4 processor NVIC ICERx register adddresses
#define NVIC_ICER0              ( (volatile uint32_t*)0xE000E180 )
#define NVIC_ICER1              ( (volatile uint32_t*)0xE000E184 )
#define NVIC_ICER2              ( (volatile uint32_t*)0xE000E188 )
#define NVIC_ICER3              ( (volatile uint32_t*)0xE000E18C )
// ARM Cortex-M4 processor NVIC priority register addresses
#define NVIC_PR_BASE_ADDR       ( (volatile uint32_t*)0xE000E400 )

// Number of priority bits implemented
#define NO_PR_BITS_IMPLEMENTED  4

/****************************************************
 *  Peripheral register definition structures       *
 ****************************************************/

// GPIO
typedef struct {
    volatile uint32_t MODER;        // GPIO port mode register                                      Address offset: 0x00
    volatile uint32_t OTYPER;       // GPIO port output typer register                              Address offset: 0x04
    volatile uint32_t OSPEEDR;      // GPIO port output speed register                              Address offset: 0x08
    volatile uint32_t PUPDR;        // GPIO port pull-up/pull-down register                         Address offset: 0x0C
    volatile uint32_t IDR;          // GPIO port input data register                                Address offset: 0x10
    volatile uint32_t ODR;          // GPIO port output data register                               Address offset: 0x14
    volatile uint32_t BSRR;         // GPIO port bit set/reset register low                         Address offset: 0x18
    volatile uint32_t LCKR;         // GPIO port configuration lock register                        Address offset: 0x1C
    volatile uint32_t AFRL;         // GPIO port alternate function low register                    Address offset: 0x20
    volatile uint32_t AFRH;         // GPIO port alternate function high register                   Address offset: 0x24
} GPIO_RegDef_t;

// RCC
typedef struct {
    volatile uint32_t CR;           // RCC clock control register                                   Address offset: 0x00
    volatile uint32_t PLL;          // RCC PLL configuration register                               Address offset: 0x04
    volatile uint32_t CFGR;         // RCC clock configuration register                             Address offset: 0x08
    volatile uint32_t CIR;          // RCC clock interrupt register                                 Address offset: 0x0C
    volatile uint32_t AHB1RSTR;     // RCC AHB1 peripheral reset register                           Address offset: 0x10
    volatile uint32_t AHB2RSTR;     // RCC AHB2 peripheral reset register                           Address offset: 0x14
    volatile uint32_t AHB3RSTR;     // RCC AHB3 peripheral reset register                           Address offset: 0x18
    uint32_t RESERVED0;             // ------------------------- RESERVED ------------------------- Address offset: 0x1C
    volatile uint32_t APB1RSTR;     // RCC APB1 peripheral reset register                           Address offset: 0x20
    volatile uint32_t APB2RSTR;     // RCC APB2 peripheral reset register                           Address offset: 0x24
    uint32_t RESERVED1;             // ------------------------- RESERVED ------------------------- Address offset: 0x28
    uint32_t RESERVED2;             // ------------------------- RESERVED ------------------------- Address offset: 0x2C
    volatile uint32_t AHB1ENR;      // RCC AHB1 peripheral clock enable register                    Address offset: 0x30
    volatile uint32_t AHB2ENR;      // RCC AHB2 peripheral clock enable register                    Address offset: 0x34
    volatile uint32_t AHB3ENR;      // RCC AHB3 peripheral clock enable register                    Address offset: 0x38
    uint32_t RESERVED3;             // ------------------------- RESERVED ------------------------- Address offset: 0x3C
    volatile uint32_t APB1ENR;      // RCC APB1 peripheral clock enable register                    Address offset: 0x40
    volatile uint32_t APB2ENR;      // RCC APB2 peripheral clock enable register                    Address offset: 0x44
    uint32_t RESERVED4;             // ------------------------- RESERVED ------------------------- Address offset: 0x48
    uint32_t RESERVED5;             // ------------------------- RESERVED ------------------------- Address offset: 0x4C
    volatile uint32_t AHB1LPENR;    // RCC AHB1 peripheral clock enable in low power mode register  Address offset: 0x50
    volatile uint32_t AHB2LPENR;    // RCC AHB2 peripheral clock enable in low power mode register  Address offset: 0x54
    volatile uint32_t AHB3LPENR;    // RCC AHB3 peripheral clock enable in low power mode register  Address offset: 0x58
    uint32_t RESERVED6;             // ------------------------- RESERVED ------------------------- Address offset: 0x5C
    volatile uint32_t APB1LPENR;    // RCC APB1 peripheral clock enable in low power mode register  Address offset: 0x60
    volatile uint32_t APB2LPENR;    // RCC APB2 peripheral clock enable in low power mode register  Address offset: 0x64
    uint32_t RESERVED7;             // ------------------------- RESERVED ------------------------- Address offset: 0x68
    uint32_t RESERVED8;             // ------------------------- RESERVED ------------------------- Address offset: 0x6C
    volatile uint32_t BDCR;         // RCC backup domain control register                           Address offset: 0x70
    volatile uint32_t CSR;          // RCC clock control & status register                          Address offset: 0x74
    uint32_t RESERVED9;             // ------------------------- RESERVED ------------------------- Address offset: 0x78
    uint32_t RESERVED10;            // ------------------------- RESERVED ------------------------- Address offset: 0x7C
    volatile uint32_t SSCGR;        // RCC spread spectrum clock generation register                Address offset: 0x80
    volatile uint32_t PLLI2SCFGR;   // RCC PLLI2S configuration register                            Address offset: 0x84
} RCC_RegDef_t;

// EXTI
typedef struct {
    volatile uint32_t IMR;          // EXTI interrupt mask register                                 Address offset: 0x00
    volatile uint32_t EMR;          // EXTI event mask register                                     Address offset: 0x04
    volatile uint32_t RTSR;         // EXTI rising trigger selection register                       Address offset: 0x08
    volatile uint32_t FTSR;         // EXTI falling trigger selection register                      Address offset: 0x0C
    volatile uint32_t SWIER;        // EXTI software interrupt event register                       Address offset: 0x10
    volatile uint32_t PR;           // EXTI pending register                                        Address offset: 0x14
} EXTI_RegDef_t;

// SYSCFG
typedef struct {
    volatile uint32_t MEMRMP;       // SYSCFG memory map register                                   Address offset: 0x00
    volatile uint32_t PMC;          // SYSCFG peripheral mode configuration register                Address offset: 0x04
    volatile uint32_t EXTICR1;      // SYSCFG external interrupt configuration register 1           Address offset: 0x08
    volatile uint32_t EXTICR2;      // SYSCFG external interrupt configuration register 2           Address offset: 0x0C
    volatile uint32_t EXTICR3;      // SYSCFG external interrupt configuration register 3           Address offset: 0x10
    volatile uint32_t EXTICR4;      // SYSCFG external interrupt configuration register 4           Address offset: 0x14
    uint32_t RESERVED0;             // ------------------------- RESERVED ------------------------- Address offset: 0x18
    uint32_t RESERVED1;             // ------------------------- RESERVED ------------------------- Address offset: 0x1C
    volatile uint32_t CMPCR;        // SYSCFG  compensation cell control register                   Address offset: 0x20
} SYSCFG_RegDef_t;

// SPI
typedef struct {
    volatile uint32_t CR1;       	// SPI control register 1 (not used in I2S mode)                Address offset: 0x00
    volatile uint32_t CR2;          // SPI control register 2						                Address offset: 0x04
    volatile uint32_t SR;	        // SPI status register			                                Address offset: 0x08
    volatile uint32_t DR;	        // SPI data register				                            Address offset: 0x0C
    volatile uint32_t CRCPR;        // SPI CRC polynomial register	                                Address offset: 0x10
    volatile uint32_t RXCRCR;       // SPI RX CRC register (not used in I2S mode)                   Address offset: 0x14
    volatile uint32_t TXCRCR;       // SPI TX CRC register (not used in I2S mode)                   Address offset: 0x18
    volatile uint32_t I2SCFGR;      // SPI I2S configuration register                               Address offset: 0x1C
    volatile uint32_t I2SPR;        // SPI I2S prescaler register 			                        Address offset: 0x20
} SPI_RegDef_t;

/****************************************************
 *  Peripheral register definitions                 *
 ****************************************************/

// GPIO
#define GPIOA                   ( (GPIO_RegDef_t*)GPIOA_BASEADDR )
#define GPIOB                   ( (GPIO_RegDef_t*)GPIOB_BASEADDR )
#define GPIOC                   ( (GPIO_RegDef_t*)GPIOC_BASEADDR )
#define GPIOD                   ( (GPIO_RegDef_t*)GPIOD_BASEADDR )
#define GPIOE                   ( (GPIO_RegDef_t*)GPIOE_BASEADDR )
#define GPIOF                   ( (GPIO_RegDef_t*)GPIOF_BASEADDR )
#define GPIOG                   ( (GPIO_RegDef_t*)GPIOG_BASEADDR )
#define GPIOH                   ( (GPIO_RegDef_t*)GPIOH_BASEADDR )
#define GPIOI                   ( (GPIO_RegDef_t*)GPIOI_BASEADDR )

// RCC
#define RCC                     ( (RCC_RegDef_t*)RCC_BASEADDR )

// EXTI
#define EXTI                    ( (EXTI_RegDef_t*)EXTI_BASEADDR )

// SYSCFG
#define SYSCFG                  ( (SYSCFG_RegDef_t*)SYSCFG_BASEADDR )

// SPI
#define SPI1					( (SPI_RegDef_t*)SPI1_BASEADDR )
#define SPI2					( (SPI_RegDef_t*)SPI2_I2S2_BASEADDR )
#define SPI3					( (SPI_RegDef_t*)SPI3_I2S3_BASEADDR )

/****************************************************
 *  Clock enable macros                             *
 ****************************************************/

// GPIO
#define ENABLE_GPIOA_CLK()      ( RCC->AHB1ENR |= ( 1 << 0 ) )
#define ENABLE_GPIOB_CLK()      ( RCC->AHB1ENR |= ( 1 << 1 ) )
#define ENABLE_GPIOC_CLK()      ( RCC->AHB1ENR |= ( 1 << 2 ) )
#define ENABLE_GPIOD_CLK()      ( RCC->AHB1ENR |= ( 1 << 3 ) )
#define ENABLE_GPIOE_CLK()      ( RCC->AHB1ENR |= ( 1 << 4 ) )
#define ENABLE_GPIOF_CLK()      ( RCC->AHB1ENR |= ( 1 << 5 ) )
#define ENABLE_GPIOG_CLK()      ( RCC->AHB1ENR |= ( 1 << 6 ) )
#define ENABLE_GPIOH_CLK()      ( RCC->AHB1ENR |= ( 1 << 7 ) )
#define ENABLE_GPIOI_CLK()      ( RCC->AHB1ENR |= ( 1 << 8 ) )

// I2C
#define ENABLE_I2C1_CLK()       ( RCC->APB1ENR |= ( 1 << 21 ) )
#define ENABLE_I2C2_CLK()       ( RCC->APB1ENR |= ( 1 << 22 ) )
#define ENABLE_I2C3_CLK()       ( RCC->APB1ENR |= ( 1 << 23 ) )

// SPI
#define ENABLE_SPI1_CLK()		( RCC->APB2ENR |= ( 1 << 12 ) )
#define ENABLE_SPI2_CLK()		( RCC->APB1ENR |= ( 1 << 14 ) )
#define ENABLE_SPI3_CLK()		( RCC->APB1ENR |= ( 1 << 15 ) )

// USART
#define ENABLE_USART1_CLK()     ( RCC->APB2ENR |= ( 1 << 4 ) )
#define ENABLE_USART2_CLK()     ( RCC->APB1ENR |= ( 1 << 17 ) )
#define ENABLE_USART3_CLK()     ( RCC->APB1ENR |= ( 1 << 18 ) )
#define ENABLE_USART6_CLK()     ( RCC->APB2ENR |= ( 1 << 5 ) )

// UART
#define ENABLE_UART4_CLK()      ( RCC->APB1ENR |= ( 1 << 19 ) )
#define ENABLE_UART5_CLK()      ( RCC->APB1ENR |= ( 1 << 20 ) )

// SYSCFG
#define ENABLE_SYSCFG_CLK()     ( RCC->APB2ENR |= ( 1 << 14 ) )

/****************************************************
 *  Clock disable macros                            *
 ****************************************************/

// GPIO
#define DISABLE_GPIOA_CLK()     ( RCC->AHB1ENR &= ~( 1 << 0 ) )
#define DISABLE_GPIOB_CLK()     ( RCC->AHB1ENR &= ~( 1 << 1 ) )
#define DISABLE_GPIOC_CLK()     ( RCC->AHB1ENR &= ~( 1 << 2 ) )
#define DISABLE_GPIOD_CLK()     ( RCC->AHB1ENR &= ~( 1 << 3 ) )
#define DISABLE_GPIOE_CLK()     ( RCC->AHB1ENR &= ~( 1 << 4 ) )
#define DISABLE_GPIOF_CLK()     ( RCC->AHB1ENR &= ~( 1 << 5 ) )
#define DISABLE_GPIOG_CLK()     ( RCC->AHB1ENR &= ~( 1 << 6 ) )
#define DISABLE_GPIOH_CLK()     ( RCC->AHB1ENR &= ~( 1 << 7 ) )
#define DISABLE_GPIOI_CLK()     ( RCC->AHB1ENR &= ~( 1 << 8 ) )

// I2C
#define DISABLE_I2C1_CLK()      ( RCC->APB1ENR &= ~( 1 << 21 ) )
#define DISABLE_I2C2_CLK()      ( RCC->APB1ENR &= ~( 1 << 22 ) )
#define DISABLE_I2C3_CLK()      ( RCC->APB1ENR &= ~( 1 << 23 ) )

// SPI
#define DISABLE_SPI1_CLK()      ( RCC->APB2ENR &= ~( 1 << 12 ) )
#define DISABLE_SPI2_CLK()		( RCC->APB1ENR &= ~( 1 << 14 ) )
#define DISABLE_SPI3_CLK()		( RCC->APB1ENR &= ~( 1 << 15 ) )

// USART
#define DISABLE_USART1_CLK()    ( RCC->APB2ENR &= ~( 1 << 4 ) )
#define DISABLE_USART2_CLK()    ( RCC->APB1ENR &= ~( 1 << 17 ) )
#define DISABLE_USART3_CLK()    ( RCC->APB1ENR &= ~( 1 << 18 ) )
#define DISABLE_USART6_CLK()    ( RCC->APB2ENR &= ~( 1 << 5 ) )

// UART
#define DISABLE_UART4_CLK()     ( RCC->APB1ENR &= ~( 1 << 19 ) )
#define DISABLE_UART5_CLK()     ( RCC->APB1ENR &= ~( 1 << 20 ) )

// SYSCFG
#define DISABLE_SYSCFG_CLK()    ( RCC->APB2ENR &= ~( 1 << 14 ) )

/****************************************************
 *  IRQ numbers                                     *
 ****************************************************/

// Table 62. Vector table for STM32F405xx/07xx and STM32F415xx/17xx of RM0090 Reference Manual
//      Acronym                     NVIC position       Address
#define IRQ_EXTI0                   6                   // 0x0000_0058
#define IRQ_EXTI1                   7                   // 0x0000_005C
#define IRQ_EXTI2                   8                   // 0x0000_0060   
#define IRQ_EXTI3                   9                   // 0x0000_0064
#define IRQ_EXTI4                   10                  // 0x0000_0068
#define IRQ_EXTI9_5                 23                  // 0x0000_009C
#define IRQ_EXTI15_10               40                  // 0x0000_00E0

// Priority values
#define NVIC_IRQ_PRIO_0             0
#define NVIC_IRQ_PRIO_1             1
#define NVIC_IRQ_PRIO_2             2
#define NVIC_IRQ_PRIO_3             3
#define NVIC_IRQ_PRIO_4             4
#define NVIC_IRQ_PRIO_5             5
#define NVIC_IRQ_PRIO_6             6
#define NVIC_IRQ_PRIO_7             7
#define NVIC_IRQ_PRIO_8             8
#define NVIC_IRQ_PRIO_9             9
#define NVIC_IRQ_PRIO_10            10
#define NVIC_IRQ_PRIO_11            11
#define NVIC_IRQ_PRIO_12            12
#define NVIC_IRQ_PRIO_13            13
#define NVIC_IRQ_PRIO_14            14
#define NVIC_IRQ_PRIO_15            15

/****************************************************
 *  GPIOx peripheral reset macros                   *
 ****************************************************/

#define GPIOA_REG_RESET()		    do { (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0)); } while(0)
#define GPIOB_REG_RESET()		    do { (RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1)); } while(0)
#define GPIOC_REG_RESET()		    do { (RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2)); } while(0)
#define GPIOD_REG_RESET()		    do { (RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3)); } while(0)
#define GPIOE_REG_RESET()		    do { (RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4)); } while(0)
#define GPIOF_REG_RESET()		    do { (RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &= ~(1 << 5)); } while(0)
#define GPIOG_REG_RESET()		    do { (RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &= ~(1 << 6)); } while(0)
#define GPIOH_REG_RESET()		    do { (RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7)); } while(0)
#define GPIOI_REG_RESET()		    do { (RCC->AHB1RSTR |= (1 << 8)); (RCC->AHB1RSTR &= ~(1 << 8)); } while(0)

#define GPIO_BASEADDR_TO_CODE(x)    (x == GPIOA) ? 0 : \
                                    (x == GPIOB) ? 1 : \
                                    (x == GPIOC) ? 2 : \
                                    (x == GPIOD) ? 3 : \
                                    (x == GPIOE) ? 4 : \
                                    (x == GPIOF) ? 5 : \
                                    (x == GPIOG) ? 6 : \
                                    (x == GPIOH) ? 7 : 0

/****************************************************
 *  Flash and SRAM                                  *
 ****************************************************/

#define FLASH_BASEADDR			0x08000000U
#define SRAM1_BASEADDR			0x20000000U		// 112 kB
#define SRAM2_BASEADDR			0x20001C00U		// 16 kB
#define ROM_BASEADDR			0x1FFF0000U
#define OTP_BASEADDR			0x1FFF7800U
#define SRAM_BASEADDR			SRAM1_BASEADDR

/****************************************************
 *  AHBx and APBx bus peripherals                   *
 ****************************************************/

#define PERIPH_BASEADDR			0x40000000U

//****************************** AHB2 ******************************//
#define AHB2PERIPH_BASEADDR		0x50000000U                         //
#define RNG_BASEADDR			( AHB2PERIPH_BASEADDR + 0x60800 )   // 0x5006_0800-0x5006_0BFF
// --------------------------- RESERVED --------------------------- // 0x5005_0400-0x5006_07FF
#define DCMI_BASEADDR			( AHB2PERIPH_BASEADDR + 0x50000 )   // 0x5005_0000-0x5005_03FF
// --------------------------- RESERVED --------------------------- // 0x5004_0000-0x5004_FFFF
#define USBOTGFS_BASEADDR		( AHB2PERIPH_BASEADDR + 0x0000 )    // 0x5000_0000-0x5003_FFFF
//******************************************************************//

// --------------------------- RESERVED --------------------------- // 0x4008_0000-0x4FFF_FFFF

//****************************** AHB1 ******************************//
#define AHB1PERIPH_BASEADDR		0x40020000U                         //
#define USBOTGHS_BASEADDR		( AHB1PERIPH_BASEADDR + 0x20000 )   // 0x4004_0000-0x4007_FFFF
// --------------------------- RESERVED --------------------------- // 0x4002_9400-0x4003_FFFF
#define EMAC_BASEADDR			( AHB1PERIPH_BASEADDR + 0x8000 )    // 0x4002_8000-0x4002_93FF
// --------------------------- RESERVED --------------------------- // 0x4002_6800-0x4002_7FFF
#define DMA2_BASEADDR			( AHB1PERIPH_BASEADDR + 0x6400 )    // 0x4002_6400-0x4002_67FF
#define DMA1_BASEADDR			( AHB1PERIPH_BASEADDR + 0x6000 )    // 0x4002_6000-0x4002_63FF
// --------------------------- RESERVED --------------------------- // 0x4002_5000-0x4002_5FFF
#define BKPSRAM_BASEADDR		( AHB1PERIPH_BASEADDR + 0x4000 )    // 0x4002_4000-0x4002_4FFF
#define FIR_BASEADDR			( AHB1PERIPH_BASEADDR + 0x3C00 )    // 0x4002_3C00-0x4002_3FFF
#define RCC_BASEADDR			( AHB1PERIPH_BASEADDR + 0x3800 )    // 0x4002_3800-0x4002_3BFF
// --------------------------- RESERVED --------------------------- // 0x4002_3400-0x4002_37FF
#define CRC_BASEADDR			( AHB1PERIPH_BASEADDR + 0x3000 )    // 0x4002_3000-0x4002_33FF
// --------------------------- RESERVED --------------------------- // 0x4002_2400-0x4002_2FFF
#define GPIOI_BASEADDR			( AHB1PERIPH_BASEADDR + 0x2000 )    // 0x4002_2000-0x4002_23FF
#define GPIOH_BASEADDR			( AHB1PERIPH_BASEADDR + 0x1C00 )    // 0x4002_1C00-0x4002_1FFF
#define GPIOG_BASEADDR			( AHB1PERIPH_BASEADDR + 0x1800 )    // 0x4002_1800-0x4002_1BFF
#define GPIOF_BASEADDR			( AHB1PERIPH_BASEADDR + 0x1400 )    // 0x4002_1400-0x4002_17FF
#define GPIOE_BASEADDR			( AHB1PERIPH_BASEADDR + 0x1000 )    // 0x4002_1000-0x4002_13FF
#define GPIOD_BASEADDR			( AHB1PERIPH_BASEADDR + 0x0C00 )    // 0x4002_0C00-0x4002_0FFF
#define GPIOC_BASEADDR			( AHB1PERIPH_BASEADDR + 0x0800 )    // 0x4002_0800-0x4002_0BFF
#define GPIOB_BASEADDR			( AHB1PERIPH_BASEADDR + 0x0400 )    // 0x4002_0400-0x4002_07FF
#define GPIOA_BASEADDR			( AHB1PERIPH_BASEADDR + 0x0000 )    // 0x4002_0000-0x4002_03FF
//******************************************************************//

// --------------------------- RESERVED --------------------------- // 0x4001_5800-0x4001_FFFF

//****************************** APB2 ******************************//
#define APB2PERIPH_BASEADDR		0x40010000U                         //
// --------------------------- RESERVED --------------------------- // 0x4000_4C00-0x4001_57FF
#define TIM11_BASEADDR			( APB2PERIPH_BASEADDR + 0x4800 )    // 0x4001_4800-0x4001_4BFF
#define TIM10_BASEADDR			( APB2PERIPH_BASEADDR + 0x4400 )    // 0x4001_4400-0x4001_47FF
#define TIM9_BASEADDR			( APB2PERIPH_BASEADDR + 0x4000 )    // 0x4001_4000-0x4001_43FF
#define EXTI_BASEADDR			( APB2PERIPH_BASEADDR + 0x3C00 )    // 0x4001_3C00-0x4001_3FFF
#define SYSCFG_BASEADDR			( APB2PERIPH_BASEADDR + 0x3800 )    // 0x4001_3800-0x4001_3BFF
// --------------------------- RESERVED --------------------------- // 0x4001_3400-0x4001_37FF
#define SPI1_BASEADDR			( APB2PERIPH_BASEADDR + 0x3000 )    // 0x4001_3000-0x4001_33FF
#define SDIO_BASEADDR			( APB2PERIPH_BASEADDR + 0x2C00 )    // 0x4001_2C00-0x4001_2FFF
// --------------------------- RESERVED --------------------------- // 0x4001_2400-0x4000_2BFF
#define ADC1_ADC2_ADC3_BASEADDR	( APB2PERIPH_BASEADDR + 0x2000 )    // 0x4001_2000-0x4001_23FF
// --------------------------- RESERVED --------------------------- // 0x4001_1800-0x4000_1FFF
#define USART6_BASEADDR			( APB2PERIPH_BASEADDR + 0x1400 )    // 0x4001_1400-0x4001_17FF
#define USART1_BASEADDR			( APB2PERIPH_BASEADDR + 0x1000 )    // 0x4001_1000-0x4001_13FF
// --------------------------- RESERVED --------------------------- // 0x4001_0800-0x4000_0FFF
#define TIM8_BASEADDR			( APB2PERIPH_BASEADDR + 0x0400 )    // 0x4001_0400-0x4001_07FF
#define TIM1_BASEADDR			( APB2PERIPH_BASEADDR + 0x0000 )    // 0x4001_0000-0x4001_03FF
//******************************************************************//

// --------------------------- RESERVED --------------------------- // 0x4000_7800-0x4000_7FFF

//****************************** APB1 ******************************//
#define APB1PERIPH_BASEADDR		PERIPH_BASEADDR                     //
#define DAC_BASEADDR			( APB1PERIPH_BASEADDR + 0x7400 )    // 0x4000_7400-0x4000_77FF
#define PWR_BASEADDR			( APB1PERIPH_BASEADDR + 0x7000 )    // 0x4000_7000-0x4000_73FF
// --------------------------- RESERVED --------------------------- // 0x4000_6C00-0x4000_6FFF
#define CAN2_BASEADDR			( APB1PERIPH_BASEADDR + 0x6800 )    // 0x4000_6800-0x4000_6BFF
#define CAN1_BASEADDR			( APB1PERIPH_BASEADDR + 0x6400 )    // 0x4000_6400-0x4000_67FF
// --------------------------- RESERVED --------------------------- // 0x4000_6000-0x4000_63FF
#define I2C3_BASEADDR			( APB1PERIPH_BASEADDR + 0x5C00 )    // 0x4000_5C00-0x4000_5FFF
#define I2C2_BASEADDR			( APB1PERIPH_BASEADDR + 0x5800 )    // 0x4000_5800-0x4000_5BFF
#define I2C1_BASEADDR			( APB1PERIPH_BASEADDR + 0x5400 )    // 0x4000_5400-0x4000_57FF
#define UART5_BASEADDR			( APB1PERIPH_BASEADDR + 0x5000 )    // 0x4000_5000-0x4000_53FF
#define UART4_BASEADDR			( APB1PERIPH_BASEADDR + 0x4C00 )    // 0x4000_4C00-0x4000_4FFF
#define USART3_BASEADDR			( APB1PERIPH_BASEADDR + 0x4800 )    // 0x4000_4800-0x4000_4BFF
#define USART2_BASEADDR			( APB1PERIPH_BASEADDR + 0x4400 )    // 0x4000_4400-0x4000_47FF
#define I2S3EXT_BASEADDR		( APB1PERIPH_BASEADDR + 0x4000 )    // 0x4000_4000-0x4000_43FF
#define SPI3_I2S3_BASEADDR		( APB1PERIPH_BASEADDR + 0x3C00 )    // 0x4000_3C00-0x4000_3FFF
#define SPI2_I2S2_BASEADDR		( APB1PERIPH_BASEADDR + 0x3800 )    // 0x4000_3800-0x4000_3BFF
#define I2S2EXT_BASEADDR		( APB1PERIPH_BASEADDR + 0x3400 )    // 0x4000_3400-0x4000_37FF
#define IWDG_BASEADDR			( APB1PERIPH_BASEADDR + 0x3000 )    // 0x4000_3000-0x4000_33FF
#define WWDG_BASEADDR			( APB1PERIPH_BASEADDR + 0x2C00 )    // 0x4000_2C00-0x4000_2FFF
#define RTC_BKP_BASEADDR		( APB1PERIPH_BASEADDR + 0x2800 )    // 0x4000_2800-0x4000_2BFF
// --------------------------- RESERVED --------------------------- // 0x4000_2400-0x4000_27FF
#define TIM14_BASEADDR			( APB1PERIPH_BASEADDR + 0x2000 )    // 0x4000_2000-0x4000_23FF
#define TIM13_BASEADDR			( APB1PERIPH_BASEADDR + 0x1C00 )    // 0x4000_1C00-0x4000_1FFF
#define TIM12_BASEADDR			( APB1PERIPH_BASEADDR + 0x1800 )    // 0x4000_1800-0x4000_1BFF
#define TIM7_BASEADDR			( APB1PERIPH_BASEADDR + 0x1400 )    // 0x4000_1400-0x4000_17FF
#define TIM6_BASEADDR			( APB1PERIPH_BASEADDR + 0x1000 )    // 0x4000_1000-0x4000_13FF
#define TIM5_BASEADDR			( APB1PERIPH_BASEADDR + 0x0C00 )    // 0x4000_0C00-0x4000_0FFF
#define TIM4_BASEADDR			( APB1PERIPH_BASEADDR + 0x0800 )    // 0x4000_0800-0x4000_0BFF
#define TIM3_BASEADDR			( APB1PERIPH_BASEADDR + 0x0400 )    // 0x4000_0400-0x4000_07FF
#define TIM2_BASEADDR			( APB1PERIPH_BASEADDR + 0x0000 )    // 0x4000_0000-0x4000_03FF
//******************************************************************//

/****************************************************
 *  Bit position definitions for SPI peripheral     *
 ****************************************************/

// CR1 register
#define SPI_CR1_CPHA            0
#define SPI_CR1_CPOL            1
#define SPI_CR1_MSTR            2
#define SPI_CR1_BR              3
#define SPI_CR1_SPE             6
#define SPI_CR1_LSBFIRST        7
#define SPI_CR1_SSI             8
#define SPI_CR1_SSM             9
#define SPI_CR1_RXONLY          10
#define SPI_CR1_DFF             11
#define SPI_CR1_CRCNEXT         12
#define SPI_CR1_CRCEN           13
#define SPI_CR1_BIDIOE          14
#define SPI_CR1_BIDIMODE        15

// CR2 register
#define SPI_CR2_RXDMAEN         0
#define SPI_CR2_TXDMAEN         1
#define SPI_CR2_SSOE            2
#define SPI_CR2_FRF             4
#define SPI_CR2_ERRIE           5
#define SPI_CR2_RXNEIE          6
#define SPI_CR2_TXEIE           7

// SR register
#define SPI_SR_RXNE             0
#define SPI_SR_TXE              1
#define SPI_SR_CHSIDE           2
#define SPI_SR_UDR              3
#define SPI_SR_CRCERR           4
#define SPI_SR_MODF             5
#define SPI_SR_OVR              6
#define SPI_SR_BSY              7
#define SPI_SR_FRE              8

#endif /* INC_STM32F407XX_H_ */
