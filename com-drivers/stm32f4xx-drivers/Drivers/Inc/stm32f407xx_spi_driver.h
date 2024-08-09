
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
	uint8_t SPI_DeviceMode;				// Possible values from @SPI_
	uint8_t SPI_BusConfig;				// Possible values from @SPI_
	uint8_t SPI_SclkSpeed;				// Possible values from @SPI_
	uint8_t SPI_DFF;					// Possible values from @SPI_
	uint8_t SPI_CPOL;					// Possible values from @SPI_
	uint8_t SPI_CPHA;					// Possible values from @SPI_
	uint8_t SPI_SSM;					// Possible values from @SPI_
} SPI_Config_t;

typedef struct {
	SPI_RegDef_t *pSPIx;				// pointer to hold the base adddress of the SPI peripheral
	SPI_Config_t SPIConfig;				// holds SPI configuration settings
} SPI_Handle_t;

/****************************************************
 *  Macros										    *
 ****************************************************/

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
