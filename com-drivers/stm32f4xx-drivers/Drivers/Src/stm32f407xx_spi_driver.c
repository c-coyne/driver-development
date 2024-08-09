
/*=====================================================================================*\
| Author:   Christopher Coyne                                         August 9th, 2024  |
| --------------------------------------------------------------------------------------|
| Date:     August 5th, 2024                                                            |
| --------------------------------------------------------------------------------------|
| MODULE:     [ SPI driver ]                                                            |
| FILE:       stm32f407xx_spi_driver.c                                                  |
| --------------------------------------------------------------------------------------|
| DESCRIPTION:                                                                          |
|    ...  		                                                                        |
\*=====================================================================================*/

/****************************************************
 *  Include files                                   *
 ****************************************************/

#include "stm32f407xx_spi_driver.h"

/****************************************************
 *  API function definitions					    *
 ****************************************************/

// Peripheral clock setup
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t En) {

	if(ENABLE == En) {
		if(SPI1 == pSPIx) {
			ENABLE_SPI1_CLK();
		}
		else if(SPI2 == pSPIx) {
			ENABLE_SPI2_CLK();
		}
		else if(SPI3 == pSPIx) {
			ENABLE_SPI3_CLK();
		}
	}
	else {
		if(SPI1 == pSPIx) {
			DISABLE_SPI1_CLK();
		}
		else if(SPI2 == pSPIx) {
			DISABLE_SPI2_CLK();
		}
		else if(SPI3 == pSPIx) {
			DISABLE_SPI3_CLK();
		}
	}

}

// Init
void SPI_Init(SPI_Handle_t *pSPIHandle) {

}

// De-Init
void SPI_DeInit(SPI_RegDef_t *pSPIx) {

}

// Data send
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t length) {

}

// Data receive
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t length) {

}

// IRQ configuration and ISR handling
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t En) {

}

void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority) {

}

void SPI_IRQHandling(SPI_Handle_t *pHandle) {

}
