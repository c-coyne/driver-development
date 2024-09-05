
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

	// Enable peripheral clock
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

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

	pSPIx->CR1 = 0;

}

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t flag) {

	if (pSPIx->SR & flag) {
		return FLAG_SET;
	}

	return FLAG_RESET;

}

// Data send
// Note, this is a blocking call (it will block while length > 0)
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t length) {

	while (length > 0) {

		// 1. Wait until TXE (transmit buffer empty) bit is set
		while (SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG)); // wait until TXE = 1 (Tx buffer empty)
	
		// 2. Check the DFF bit in CR1 register
		if (pSPIx->CR1 & ( 1 << SPI_CR1_DFF )) {
			
			// 16-bit DFF
			// 2a. Load the data into the data register (DR)
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			length--; // accounts for first byte of data sent
			length--; // accounts for second byte of data sent

			// 2b. Increment pTxBuffer pointer
			(uint16_t*)pTxBuffer++; // increment by two bytes

		}
		else {

			// 8-bit DFF
			// 2a. Load the data into the data register (DR)
			pSPIx->DR = *(pTxBuffer);
			length--; // accounts for byte of data sent

			// 2b. Increment pTxBuffer pointer
			pTxBuffer++; // increment by one byte

		}
	
	}

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

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t En) {

	if (En == ENABLE) {
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	}
	else {
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}

}

void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t En) {

	if (En == ENABLE) {
		pSPIx->CR1 |= (1 << SPI_CR1_SSI);
	}
	else {
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
	}

}