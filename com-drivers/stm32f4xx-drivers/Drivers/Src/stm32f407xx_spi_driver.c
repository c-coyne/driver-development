
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

/*******************************************************************************************************
 * @brief Controls the clock for the specified SPI peripheral.                                         *
 *                                                                                                     *
 * This function enables or disables the clock for the specified SPI peripheral by controlling the     *
 * associated peripheral clock register.                                                               *
 *                                                                                                     *
 * @param pSPIx [SPI_RegDef_t*] Pointer to the SPI peripheral base address.                            *
 * @param En [uint8_t] ENABLE (1) to enable the clock, DISABLE (0) to disable the clock.               *
 *                                                                                                     *
 * @return None                                                                                        *
 *                                                                                                     *
 * @note This function works with the SPI1, SPI2, and SPI3 peripherals. For each, the clock can be     *
 *       enabled or disabled by calling the corresponding macros (`ENABLE_SPIx_CLK()` or               *
 *       `DISABLE_SPIx_CLK()`).                                                                        *
 * @note Ensure that the correct peripheral is passed in as `pSPIx` (SPI1, SPI2, or SPI3). Passing an  *
 *       invalid peripheral will result in no clock changes.                                           *
 ******************************************************************************************************/
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

/*******************************************************************************************************
 * @brief Initializes the SPI peripheral with the specified configurations.                            *
 *                                                                                                     *
 * This function configures and initializes the SPI peripheral based on the settings provided in the   *
 * SPI handle. It enables the peripheral clock and sets up the device mode, bus configuration, clock   *
 * speed, data frame format, clock polarity, clock phase, and software slave management.               *
 *                                                                                                     *
 * @param pSPIHandle [SPI_Handle_t*] Pointer to the SPI handle structure containing the base address   *
 *        of the SPI peripheral and the desired configuration settings.                                *
 *                                                                                                     *
 * @return None                                                                                        *
 *                                                                                                     *
 * @note This function must be called before using the SPI peripheral for communication. It first      *
 *       enables the peripheral clock using the `SPI_PeriClockControl()` function.                     *
 * @note The function configures the following:                                                        *
 *       - Device mode (Master/Slave) via the MSTR bit in the SPI_CR1 register.                        *
 *       - Bus configuration (Full-duplex, Half-duplex, Simplex) based on the BIDIMODE and RXONLY bits.*
 *       - Serial clock speed (SCLK) via the baud rate control bits (BR[2:0]).                         *
 *       - Data frame format (8-bit or 16-bit) via the DFF bit.                                        *
 *       - Clock polarity (CPOL) and clock phase (CPHA) via the respective bits in SPI_CR1.            *
 *       - Software slave management (SSM) via the SSM bit.                                            *
 ******************************************************************************************************/
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

/*******************************************************************************************************
 * @brief De-initializes the specified SPI peripheral.                                                 *
 *                                                                                                     *
 * This function resets the SPI peripheral's control register (CR1) to its default state, effectively  *
 * disabling the peripheral and clearing any configuration that was previously set.                    *
 *                                                                                                     *
 * @param pSPIx [SPI_RegDef_t*] Pointer to the SPI peripheral base address.                            *
 *                                                                                                     *
 * @return None                                                                                        *
 *                                                                                                     *
 * @note After calling this function, the SPI peripheral will be in its default reset state, and all   *
 *       configuration registers will need to be set again before the peripheral can be re-enabled.    *
 * @note The function only resets the SPI_CR1 register, which controls the core SPI settings such as   *
 *       the device mode, bus configuration, and clock settings. Other registers remain unchanged.     *
 ******************************************************************************************************/
void SPI_DeInit(SPI_RegDef_t *pSPIx) {

	pSPIx->CR1 = 0;

}

/*******************************************************************************************************
 * @brief Checks the status of a specific flag in the SPI status register (SR).                        *
 *                                                                                                     *
 * This function reads the SPI status register (SR) and checks if the specified flag is set or reset.  *
 * The flag can be any of the SPI status flags, such as the TXE (Transmit buffer empty) or RXNE        *
 * (Receive buffer not empty) flag.                                                                    *
 *                                                                                                     *
 * @param pSPIx [SPI_RegDef_t*] Pointer to the SPI peripheral base address.                            *
 * @param flag [uint32_t] The specific status flag to check (e.g., TXE, RXNE).                         *
 *                                                                                                     *
 * @return [uint8_t] Returns FLAG_SET if the flag is set, otherwise returns FLAG_RESET.                *
 *                                                                                                     *
 * @note This function is typically used to check if the SPI peripheral is ready for data transmission *
 *       or reception, by checking flags like TXE or RXNE.                                             *
 * @note Flags must be passed in as pre-defined macros (e.g., `SPI_SR_TXE` or `SPI_SR_RXNE`)           *
 *       corresponding to the desired status bits in the SPI_SR register.                              *
 ******************************************************************************************************/
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t flag) {

	if (pSPIx->SR & flag) {
		return FLAG_SET;
	}

	return FLAG_RESET;

}

/*******************************************************************************************************
 * @brief Sends data through the SPI peripheral (blocking call).                                       *
 *                                                                                                     *
 * This function sends data through the SPI peripheral in a blocking manner, meaning it will wait      *
 * until all the data is transmitted. The function handles both 8-bit and 16-bit data frame formats    *
 * depending on the DFF bit in the SPI_CR1 register.                                                   *
 *                                                                                                     *
 * @param pSPIx [SPI_RegDef_t*] Pointer to the SPI peripheral base address.                            *
 * @param pTxBuffer [uint8_t*] Pointer to the transmit buffer containing the data to be sent.          *
 * @param length [uint32_t] Length of data (in bytes) to be sent.                                      *
 *                                                                                                     *
 * @return None                                                                                        *
 *                                                                                                     *
 * @note This is a blocking call, meaning the function will block execution until the entire data      *
 *       transmission is completed (i.e., until `length` reaches 0). It waits for the TXE (Transmit    *
 *       buffer empty) flag to be set before loading data into the SPI data register (DR).             *
 * @note If the DFF (Data Frame Format) bit in the SPI_CR1 register is set, the function sends data in *
 *       16-bit format; otherwise, it sends data in 8-bit format. The function also appropriately      *
 *       increments the transmit buffer pointer based on the data frame format.                        *
 * @note This function should only be used when the SPI is configured in a master or slave mode where  *
 *       the SPI peripheral is actively involved in transmitting data.                                 *
 ******************************************************************************************************/
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

/*******************************************************************************************************
 * @brief Enables or disables the SPI peripheral.                                                      *
 *                                                                                                     *
 * This function controls the SPI peripheral by enabling or disabling it through the SPE (SPI Enable)  *
 * bit in the SPI_CR1 register.                                                                        *
 *                                                                                                     *
 * @param pSPIx [SPI_RegDef_t*] Pointer to the SPI peripheral base address.                            *
 * @param En [uint8_t] ENABLE to enable the SPI peripheral, DISABLE to disable it.                     *
 *                                                                                                     *
 * @return None                                                                                        *
 *                                                                                                     *
 * @note Enabling the SPI peripheral by setting the SPE bit allows it to participate in data transfer  *
 *       operations. Disabling the peripheral will stop communication and put the SPI into a disabled  *
 *       state.                                                                                        *
 * @note This function should be used after configuring the SPI peripheral and before initiating any   *
 *       data transfers.                                                                               *
 ******************************************************************************************************/
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t En) {

	if (En == ENABLE) {
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	}
	else {
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}

}

/*******************************************************************************************************
 * @brief Configures the internal slave select (SSI) bit of the SPI peripheral.                        *
 *                                                                                                     *
 * This function sets or clears the SSI (Internal Slave Select) bit in the SPI_CR1 register, which     *
 * is used in master mode to ensure proper SPI operation when the SSM (Software Slave Management)      *
 * bit is enabled.                                                                                     *
 *                                                                                                     *
 * @param pSPIx [SPI_RegDef_t*] Pointer to the SPI peripheral base address.                            *
 * @param En [uint8_t] ENABLE to set the SSI bit, DISABLE to clear it.                                 *
 *                                                                                                     *
 * @return None                                                                                        *
 *                                                                                                     *
 * @note The SSI bit must be set in master mode when the SSM bit is enabled to avoid potential         *
 *       MODF (Mode Fault) errors, as it simulates the NSS (Slave Select) signal being high.           *
 * @note This function is used primarily in master mode SPI configurations where software management   *
 *       of the NSS pin is desired (i.e., when the SSM bit is set in SPI_CR1).                         *
 ******************************************************************************************************/
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t En) {

	if (En == ENABLE) {
		pSPIx->CR1 |= (1 << SPI_CR1_SSI);
	}
	else {
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
	}

}