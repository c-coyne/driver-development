
/*=====================================================================================*\
| Author:   Christopher Coyne                                         August 5th, 2024  |
| --------------------------------------------------------------------------------------|
| Date:     August 5th, 2024                                                            |
| --------------------------------------------------------------------------------------|
| MODULE:     [ SPI driver ]                                                            |
| FILE:       stm32f407xx_spi_driver.h                                                  |
| --------------------------------------------------------------------------------------|
| DESCRIPTION:                                                                          |
|    ...  		                                                                        |
\*=====================================================================================*/

#ifndef INC_STM32F407XX_SPI_DRIVER_H_
#define INC_STM32F407XX_SPI_DRIVER_H_

/****************************************************
 *  Include files                                   *
 ****************************************************/

#include "stm32f407xx.h"

/****************************************************
 *  Structures									    *
 ****************************************************/

typedef struct {
	uint8_t SPI_DeviceMode;				// Possible values from @SPI_DeviceMode
	uint8_t SPI_BusConfig;				// Possible values from @SPI_BusConfig
	uint8_t SPI_SclkSpeed;				// Possible values from @SPI_SclkSpeed
	uint8_t SPI_DFF;					// Possible values from @SPI_DFF
	uint8_t SPI_CPOL;					// Possible values from @SPI_CPOL
	uint8_t SPI_CPHA;					// Possible values from @SPI_CPHA
	uint8_t SPI_SSM;					// Possible values from @SPI_SSM
} SPI_Config_t;

typedef struct {
	SPI_RegDef_t *pSPIx;				// pointer to hold the base adddress of the SPI peripheral
	SPI_Config_t SPIConfig;				// holds SPI configuration settings
} SPI_Handle_t;

/****************************************************
 *  Macros										    *
 ****************************************************/

// @SPI_DeviceMode
// SPI device modes, MSTR register
#define SPI_DEVICE_MODE_SLAVE			0		// Default, slave configuration
#define SPI_DEVICE_MODE_MASTER			1		// Master configuration

// @SPI_BusConfig
// SPI bus configuration modes, BIDIMODE and BIDIOE registers
#define SPI_BUS_CONFIG_FD				0		// Full duplex
#define SPI_BUS_CONFIG_HD				1		// Half duplex
#define SPI_BUS_CONFIG_S_RXONLY			2		// Simplex Rx only, forced clock signal despite no MOSI data transmission

// @SPI_SclkSpeed
// SPI clock speed
#define SPI_SCLK_SPEED_DIV2				0		// [Default] fPCLK (16 MHz) / 2 = 8 MHz
#define SPI_SCLK_SPEED_DIV4				1		// fPCLK (16 MHz) / 4 = 4 MHz
#define SPI_SCLK_SPEED_DIV8				2		// fPCLK (16 MHz) / 8 = 2 MHz
#define SPI_SCLK_SPEED_DIV16			3		// fPCLK (16 MHz) / 16 = 1 MHz
#define SPI_SCLK_SPEED_DIV32			4		// fPCLK (16 MHz) / 32 = 500 kHz
#define SPI_SCLK_SPEED_DIV64			5		// fPCLK (16 MHz) / 64 = 250 kHz
#define SPI_SCLK_SPEED_DIV128			6		// fPCLK (16 MHz) / 128 = 125 kHz
#define SPI_SCLK_SPEED_DIV256			7		// fPCLK (16 MHz) / 256 = 62.5 kHz

// @SPI_DFF
// SPI Data frame format
#define SPI_DFF_8BITS					0		// [Default] 8-bit data frame format is selected for transmission / reception
#define SPI_DFF_16BITS					1		// 16-bit data frame format is selected for transmission / reception

// @SPI_CPOL
// SPI clock polarity
#define SPI_CPOL_LOW					0		// [Default] set clock to 0 when idle
#define SPI_CPOL_HIGH					1		// Set clock to 1 when idle

// @SPI_CPHA
// SPI clock phase
#define SPI_CPHA_LOW					0		// [Default] the first clock transition is the first data capture edge
#define SPI_CPHA_HIGH					1		// The second clock transition is the first data capture edge

// @SPI_SSM
// SPI software slave management
#define SPI_SSM_DISABLED				0		// [Default] software slave management disabled (i.e. hardware slave management)
#define SPI_SSM_ENABLED					1		// Software slave management enabled

/****************************************************
 *  API prototypes								    *
 ****************************************************/
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t En);
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t length);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t length);
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t En);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pHandle);


#endif /* INC_STM32F407XX_SPI_DRIVER_H_ */
