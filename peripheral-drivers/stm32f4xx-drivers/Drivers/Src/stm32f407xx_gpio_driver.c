
/*=====================================================================================*\
| Author:   Christopher Coyne                                          July 26th, 2024  |
| --------------------------------------------------------------------------------------|
| Date:     July 26th, 2024                                                             |
| --------------------------------------------------------------------------------------|
| MODULE:     [ GPIO driver ]                                                           |
| FILE:       stm32f407xx_gpio_driver.c                                                 |
| --------------------------------------------------------------------------------------|
| DESCRIPTION:                                                                          |
|    ...  		                                                                        |
\*=====================================================================================*/


/****************************************************
 *  Include files                                   *
 ****************************************************/

#include "stm32f407xx_gpio_driver.h"

/****************************************************
 *  API function definitions					    *
 ****************************************************/

/*******************************************************************************************************
 * @brief Initializes a GPIO pin on the STM32F407G-DISC1 development board.							   *
 * 																									   *
 * This function configures various settings for a specified GPIO pin based on the parameters provided *
 * in the GPIO handle structure. The function supports configuring the pin mode, speed, pull-up / 	   *
 * pull-down settings, output type, and alternate functionality.									   *
 * 																									   *
 * @param pGPIOHandle [GPIO_Handle_t*] Pointer to the GPIO handle structure containing configuration   *
 *                                     settings for the pin.										   *
 * 																									   *
 * @note The function handles different GPIO modes, including input, output, analog, and interrupt 	   *
 *       modes. It also sets up external interrupt configurations if the pin is configured for 		   *
 * 		 interrupt mode.																			   *
 ******************************************************************************************************/
void GPIO_Init(GPIO_Handle_t *pGPIOHandle) {

	uint32_t temp = 0;

	// Configure mode of GPIO pin
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG) {
		temp = ( pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
		pGPIOHandle->pGPIOx->MODER &= ~( 0x03 << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber ) );	// clear register
		pGPIOHandle->pGPIOx->MODER |= temp;	// set register
	}
	// Configure interrupt settings if mode is GPIO_MODE_INT_FT, GPIO_MODE_INT_RT, or GPIO_MODE_INT_RFT
	else {
		if(GPIO_MODE_INT_FT == pGPIOHandle->GPIO_PinConfig.GPIO_PinMode) {
			// Configure the falling trigger selection register (FTSR)
			EXTI->FTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
			// Clear the corresponding RTSR bit
			EXTI->RTSR &= ~( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
		}
		else if(GPIO_MODE_INT_RT == pGPIOHandle->GPIO_PinConfig.GPIO_PinMode) {
			// Configure the rising trigger selection register (RTSR)
			EXTI->RTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
			// Clear the corresponding FTSR bit
			EXTI->FTSR &= ~( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
		}
		else if(GPIO_MODE_INT_RFT == pGPIOHandle->GPIO_PinConfig.GPIO_PinMode) {
			// Configure both FTSR and RTSR
			EXTI->FTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
			EXTI->RTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
		}

		// Configure the GPIO port selection in SYSCFG_EXTICR
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);

		// Enable the clock
		ENABLE_SYSCFG_CLK();

		// Configure EXTI line selection for GPIO pin
		switch(temp1) {
			case 0:
				SYSCFG->EXTICR1 = ( portcode << (temp2 * 4) );
				break;
			case 1:
				SYSCFG->EXTICR2 = ( portcode << (temp2 * 4) );
				break;
			case 2:
				SYSCFG->EXTICR3 = ( portcode << (temp2 * 4) );
				break;
			case 3:
				SYSCFG->EXTICR4 = ( portcode << (temp2 * 4) );
				break;
			default:
				break;
		}

		// Enable the EXTI interrupt delivery using the interrupt mask register (IMR)
		EXTI->IMR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
	}

	temp = 0;

	// Configure speed of GPIO pin
	temp = ( pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
	pGPIOHandle->pGPIOx->OSPEEDR &= ~( 0x03 << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber ) );		// clear register
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;	// set register

	temp = 0;

	// Configure the pull-up / pull-down settings
	temp = ( pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
	pGPIOHandle->pGPIOx->PUPDR &= ~( 0x03 << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber ) );		// clear register
	pGPIOHandle->pGPIOx->PUPDR |= temp;		// set register

	temp = 0;

	// Configure the output type
	if(GPIO_OP_TYPE_PP == pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType || GPIO_OP_TYPE_OD == pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType) {
		temp = ( pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
		pGPIOHandle->pGPIOx->OTYPER &= ~( 0x01 << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber ) );		// clear register
		pGPIOHandle->pGPIOx->OTYPER |= temp;	// set register

		temp = 0;
	}

	// Configure the alternate functionality
	if(GPIO_MODE_ALTFN == pGPIOHandle->GPIO_PinConfig.GPIO_PinMode) {
		uint8_t temp1, temp2;

		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;

		switch(temp1) {
			case 0:
				pGPIOHandle->pGPIOx->AFRL &= ~( 0xFF << ( 4 * temp2 ) );
				pGPIOHandle->pGPIOx->AFRL |= ( pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << ( 4 * temp2 ) );
				break;
			case 1:
				pGPIOHandle->pGPIOx->AFRH &= ~( 0xFF << ( 4 * temp2 ) );
				pGPIOHandle->pGPIOx->AFRH |= ( pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << ( 4 * temp2 ) );
				break;
			default:
				break;
		}
	}
}

/*******************************************************************************************************
 * @brief De-initializes the specified GPIO port.													   *
 * 																									   *
 * This function resets the registers of the specified GPIO port to their default reset values. It 	   *
 * identifies the port based on the provided GPIO port base address and performs a reset operation 	   *
 * accordingly.																						   *
 * 																									   *
 * @param pGPIOx [GPIO_RegDef_t*] Pointer to the GPIO port base address to be de-initialized.		   *
 * 																									   *
 * @note This function supports de-initialization of GPIO ports A through I.						   *
 ******************************************************************************************************/
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx) {

	if(GPIOA == pGPIOx) {
		GPIOA_REG_RESET();
	}
	else if(GPIOB == pGPIOx) {
		GPIOB_REG_RESET();
	}
	else if(GPIOC == pGPIOx) {
		GPIOC_REG_RESET();
	}
	else if(GPIOD == pGPIOx) {
		GPIOD_REG_RESET();
	}
	else if(GPIOE == pGPIOx) {
		GPIOE_REG_RESET();
	}
	else if(GPIOF == pGPIOx) {
		GPIOF_REG_RESET();
	}
	else if(GPIOG == pGPIOx) {
		GPIOG_REG_RESET();
	}
	else if(GPIOH == pGPIOx) {
		GPIOH_REG_RESET();
	}
	else if(GPIOI == pGPIOx) {
		GPIOI_REG_RESET();
	}

}

/*******************************************************************************************************
 * @brief Controls the peripheral clock for the specified GPIO port.								   *
 * 																									   *
 * This function enables or disables the peripheral clock for the specified GPIO port based on the 	   *
 * provided GPIO port base address and the enable/disable command.									   *
 * 																									   *
 * @param pGPIOx [GPIO_RegDef_t*] Pointer to the GPIO port base address.							   *
 * @param En [uint8_t] Enable or disable command for the peripheral clock. Use ENABLE to turn on the   *
 *                     clock and DISABLE to turn off the clock.										   *
 * 																									   *
 * @note This function supports clock control for GPIO ports A through I.							   *
 ******************************************************************************************************/
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t En) {

	if(ENABLE == En) {
		if(GPIOA == pGPIOx) {
			ENABLE_GPIOA_CLK();
		}
		else if(GPIOB == pGPIOx) {
			ENABLE_GPIOB_CLK();
		}
		else if(GPIOC == pGPIOx) {
			ENABLE_GPIOC_CLK();
		}
		else if(GPIOD == pGPIOx) {
			ENABLE_GPIOD_CLK();
		}
		else if(GPIOE == pGPIOx) {
			ENABLE_GPIOE_CLK();
		}
		else if(GPIOF == pGPIOx) {
			ENABLE_GPIOF_CLK();
		}
		else if(GPIOG == pGPIOx) {
			ENABLE_GPIOG_CLK();
		}
		else if(GPIOH == pGPIOx) {
			ENABLE_GPIOH_CLK();
		}
		else if(GPIOI == pGPIOx) {
			ENABLE_GPIOI_CLK();
		}
	} 
	else {
		if(GPIOA == pGPIOx) {
			DISABLE_GPIOA_CLK();
		}
		else if(GPIOB == pGPIOx) {
			DISABLE_GPIOB_CLK();
		}
		else if(GPIOC == pGPIOx) {
			DISABLE_GPIOC_CLK();
		}
		else if(GPIOD == pGPIOx) {
			DISABLE_GPIOD_CLK();
		}
		else if(GPIOE == pGPIOx) {
			DISABLE_GPIOE_CLK();
		}
		else if(GPIOF == pGPIOx) {
			DISABLE_GPIOF_CLK();
		}
		else if(GPIOG == pGPIOx) {
			DISABLE_GPIOG_CLK();
		}
		else if(GPIOH == pGPIOx) {
			DISABLE_GPIOH_CLK();
		}
		else if(GPIOI == pGPIOx) {
			DISABLE_GPIOI_CLK();
		}
	}

}

/*******************************************************************************************************
 * @brief Reads the value from a specified input pin of a GPIO port.								   *
 * 																									   *
 * This function reads the current value of a specified input pin from the input data register 		   *
 * (IDR) of the provided GPIO port and returns it.													   *
 * 																									   *
 * @param pGPIOx [GPIO_RegDef_t*] Pointer to the GPIO port base address.							   *
 * @param pinNumber [uint8_t] Pin number from which the value is to be read.						   *
 * @return uint8_t Value read from the specified GPIO input pin (0 or 1).							   *
 * 																									   *
 * @note This function reads a single bit from the specified pin number in the input data register.	   *
 ******************************************************************************************************/
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber) {

	uint8_t value;
	value = (uint8_t)( (pGPIOx->IDR >> pinNumber) & 0x00000001 );
	return value;

}

/*******************************************************************************************************
 * @brief Reads the value from the input port of a GPIO port.										   *
 * 																									   *
 * This function reads the current 16-bit value from the input data register (IDR) of the provided 	   *
 * GPIO port and returns it.																		   *
 * 																									   *
 * @param pGPIOx [GPIO_RegDef_t*] Pointer to the GPIO port base address.							   *
 * @return uint16_t Value read from the input data register of the specified GPIO port.				   *
 * 																									   *
 * @note This function reads the entire 16-bit value from the input data register.					   *
 ******************************************************************************************************/
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx) {

	uint16_t value;
	value = (uint16_t)(pGPIOx->IDR);
	return value;

}

/*******************************************************************************************************
 * @brief Writes a value to a specified output pin of a GPIO port.									   *
 * 																									   *
 * This function sets or clears the output value of a specified pin in the output data register (ODR)  *
 * of the provided GPIO port.																		   *
 * 																								       *	
 * @param pGPIOx [GPIO_RegDef_t*] Pointer to the GPIO port base address.							   *
 * @param pinNumber [uint8_t] Pin number to which the value is to be written.						   *
 * @param Value [uint8_t] Value to be written to the pin (GPIO_PIN_SET or GPIO_PIN_RESET).			   *
 * 																									   *
 * @note This function writes a single bit to the specified pin number in the output data register.	   *
 ******************************************************************************************************/
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber, uint8_t Value) {

	if(GPIO_PIN_SET == Value) {
		// write 1 to output data register at the bit field corresponding to the pin
		pGPIOx->ODR |= ( 1 << pinNumber );
	}
	else {
		// write 0
		pGPIOx->ODR &= ~( 1 << pinNumber );
	}

}

/*******************************************************************************************************
 * @brief Writes a value to the output port of a GPIO port.											   *
 * 																									   *
 * This function writes a 16-bit value to the output data register (ODR) of the provided GPIO port.	   *
 * 																									   *
 * @param pGPIOx [GPIO_RegDef_t*] Pointer to the GPIO port base address.							   *
 * @param Value [uint16_t] 16-bit value to be written to the output data register.					   *
 * 																									   *
 * @note This function sets the entire 16-bit output data register.									   *
 ******************************************************************************************************/
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value) {

	pGPIOx->ODR = Value;

}

/*******************************************************************************************************
 * @brief Toggles the value of a specified output pin of a GPIO port.								   *
 * 																									   *
 * This function toggles the current output value of a specified pin in the output data register 	   *
 * (ODR) of the provided GPIO port.																	   *
 * 																									   *
 * @param pGPIOx [GPIO_RegDef_t*] Pointer to the GPIO port base address.						       *
 * @param pinNumber [uint8_t] Pin number to be toggled.												   *
 * 																									   *
 * @note This function flips the current state of the specified pin (from 0 to 1 or from 1 to 0).	   *
 ******************************************************************************************************/
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber) {

	pGPIOx->ODR ^= ( 1 << pinNumber );

}

/*******************************************************************************************************
 * @brief Configures the interrupt for a given IRQ number.											   *
 * 																									   *
 * This function enables or disables the specified IRQ number in the NVIC (Nested Vectored Interrupt   *
 * Controller) based on the provided enable/disable command.										   *
 * 																									   *
 * @param IRQNumber [uint8_t] IRQ number to be configured.											   *
 * @param En [uint8_t] Enable or disable command for the IRQ (ENABLE to enable the interrupt, DISABLE  *
 *                     to disable the interrupt).													   *
 * 																									   *
 * @note This function handles IRQ numbers 0 to 95, programming the appropriate ISER (Interrupt 	   *
 * 		 Set-Enable Register) or ICER (Interrupt Clear-Enable Register) register based on the IRQ 	   *
 * 		 number.																					   *
 ******************************************************************************************************/
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t En) {

	if(ENABLE == En) {
		if(IRQNumber <= 31) {
			// Program ISER0 register
			*NVIC_ISER0 |= ( 1 << IRQNumber );
		}
		else if(IRQNumber > 31 && IRQNumber <= 63) {
			// Program ISER1 register
			*NVIC_ISER1 |= ( 1 << (IRQNumber % 32) );
		}
		else if(IRQNumber > 63 && IRQNumber <= 95) {
			// Program ISER2 register
			*NVIC_ISER2 |= ( 1 << (IRQNumber % 64) );
		}
	}
	else {
		if(IRQNumber <= 31) {
			// Program ICER0 register
			*NVIC_ICER0 |= ( 1 << IRQNumber );
		}
		else if(IRQNumber > 31 && IRQNumber <= 63) {
			// Program ICER1 register
			*NVIC_ICER1 |= ( 1 << (IRQNumber % 32) );
		}
		else if(IRQNumber > 63 && IRQNumber <= 95) {
			// Program ICER2 register
			*NVIC_ICER2 |= ( 1 << (IRQNumber % 64) );
		}
	}

}

/*******************************************************************************************************
 * @brief Configures the priority of a given IRQ number in the NVIC.								   *
 * 																									   *
 * This function sets the priority level for a specified IRQ number based on the provided priority 	   *
 * value. It determines the appropriate NVIC priority register and updates it accordingly.			   *
 * 																									   *
 * @param IRQNumber [uint8_t] IRQ number for which the priority is to be configured.				   *
 * @param IRQPriority [uint32_t] Priority level to be set for the IRQ number.						   *
 * 																									   *
 * @note This function calculates the appropriate NVIC priority register and the bit shift amount 	   *
 * 		 based on the IRQ number and priority configuration.										   *
 ******************************************************************************************************/
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority) {

	// Determine the proper IPR register
	uint8_t IPRx = IRQNumber / 4;
	uint8_t IPRxSection = IRQNumber % 4;

	// Program the proper register
	uint8_t shiftAmount = (8 * IPRxSection) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + IPRx) |= (IRQPriority << shiftAmount);

}

/*******************************************************************************************************
 * @brief Handles the IRQ for a specified GPIO pin by clearing the pending bit.						   *
 * 																									   *
 * This function checks the EXTI (External Interrupt) pending register for the specified pin number.   *
 * If the bit corresponding to the pin number is set, it clears the pending bit to acknowledge the 	   *
 * interrupt.																						   *
 * 																									   *
 * @param PinNumber [uint8_t] GPIO pin number for which the interrupt handling is to be performed.	   *
 * 																									   *
 * @note This function ensures that the pending interrupt request for the specified pin is cleared 	   *
 * 		 from the EXTI pending register.															   *
 ******************************************************************************************************/
void GPIO_IRQHandling(uint8_t PinNumber) {

	// Check for pended EXTI PR register bit corresponding to the pin number
	if(EXTI->PR & ( 1 << PinNumber )) {
		// Clear the pended bit
		EXTI->PR |= ( 1 << PinNumber );
	}

}

/*******************************************************************************************************
 * @brief Locks the configuration of a specified GPIO pin.                                             *
 *                                                                                                     *
 * This function locks the configuration of the specified GPIO pin by performing the lock key sequence *
 * on the GPIO port's lock register (LCKR). Once a pin is locked, its configuration cannot be modified *
 * until the next reset.                                                                               *
 *                                                                                                     *
 * @param pGPIOx [GPIO_RegDef_t*] Pointer to the GPIO port base address.			                   *
 * @param pinNumber [uint8_t] GPIO pin number to be locked.                                            *
 *                                                                                                     *
 * @return [uint8_t] Returns 1 if the pin is successfully locked, otherwise returns 0.                 *
 *                                                                                                     *
 * @note The lock key sequence involves writing a specific pattern to the GPIO port's lock register,   *
 *       followed by a dummy read to complete the sequence. This function ensures that the specified   *
 *       pin's configuration is locked by checking the lock bit (LCKK) in the lock register.           *
 ******************************************************************************************************/
uint8_t GPIO_LockPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber)
{
    // Create the lock key sequence
    uint32_t lockKey = 0x00010000 | pinNumber;

    // Write the lock key sequence
    pGPIOx->LCKR = lockKey;
    pGPIOx->LCKR = pinNumber;
    pGPIOx->LCKR = lockKey;
    (void)pGPIOx->LCKR; // Dummy read to complete the sequence

    // Check if the pin is locked
    if (pGPIOx->LCKR & GPIO_LCKR_LCKK)
    {
        return 1; // Locked
    }
    else
    {
        return 0; // Unlocked
    }
}