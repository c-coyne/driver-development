
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

	// First configure SPI_CR1 register
	uint32_t tempreg = 0;

	// 1. Configure the device mode (MSTR master selection register)
	tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;

	// 2. Configure the bus configuration
	if(SPI_BUS_CONFIG_FD == pSPIHandle->SPIConfig.SPI_BusConfig) {
		// BIDI mode should be cleared
		tempreg &= ~( 1 << SPI_CR1_BIDIMODE );
	} 
	else if (SPI_BUS_CONFIG_HD == pSPIHandle->SPIConfig.SPI_BusConfig) {
		// BIDI mode should be enabled
		tempreg |= ( 1 << SPI_CR1_BIDIMODE );
	}
	else if (SPI_BUS_CONFIG_S_RXONLY == pSPIHandle->SPIConfig.SPI_BusConfig) {
		// BIDI mode should be cleared
		tempreg &= ~( 1 << SPI_CR1_BIDIMODE );
		// RXONLY bit should be set
		tempreg |= ( 1 << SPI_CR1_RXONLY );
	}

	// 3. Configure the SPI serial clock speed
	tempreg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;

	// 4. Configure the DFF
	tempreg |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;

	// 5. Configure the CPOL
	tempreg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;

	// 6. Configure the CPHA
	tempreg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

	// 7. Configure the SSM
	tempreg |= pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM;

	// Assign the configuration to the CR1 register
	pSPIHandle->pSPIx->CR1 = tempreg;

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
