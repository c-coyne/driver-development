
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

	// 1. Configure mode of GPIO pin
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG) {
		temp = ( pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
		pGPIOHandle->pGPIOx->MODER &= ~( 0x03 << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber ) );	// clear register
		pGPIOHandle->pGPIOx->MODER |= temp;	// set register
	}
	else {
		if(GPIO_MODE_INT_FT == pGPIOHandle->GPIO_PinConfig.GPIO_PinMode) {
			// 1. Configure the falling trigger selection register (FTSR)
			EXTI->FTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
			// Clear the corresponding RTSR bit
			EXTI->RTSR &= ~( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
		}
		else if(GPIO_MODE_INT_RT == pGPIOHandle->GPIO_PinConfig.GPIO_PinMode) {
			// 1. Configure the rising trigger selection register (RTSR)
			EXTI->RTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
			// Clear the corresponding FTSR bit
			EXTI->FTSR &= ~( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
		}
		else if(GPIO_MODE_INT_RFT == pGPIOHandle->GPIO_PinConfig.GPIO_PinMode) {
			// 1. Configure both FTSR and RTSR
			EXTI->FTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
			EXTI->RTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
		}
		// 2. Configure the GPIO port selection in SYSCFG_EXTICR
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);

		ENABLE_SYSCFG_CLK();

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
		// 3. Enable the EXTI interrupt delivery using the interrupt mask register (IMR)
		EXTI->IMR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
	}

	temp = 0;

	// 2. Configure speed of GPIO pin
	temp = ( pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
	pGPIOHandle->pGPIOx->OSPEEDR &= ~( 0x03 << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber ) );		// clear register
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;	// set register

	temp = 0;

	// 3. Configure the pull-up / pull-down settings
	temp = ( pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
	pGPIOHandle->pGPIOx->PUPDR &= ~( 0x03 << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber ) );		// clear register
	pGPIOHandle->pGPIOx->PUPDR |= temp;		// set register

	temp = 0;

	// 4. Configure the output type
	if(GPIO_OP_TYPE_PP == pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType || GPIO_OP_TYPE_OD == pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType) {
		temp = ( pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
		pGPIOHandle->pGPIOx->OTYPER &= ~( 0x01 << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber ) );		// clear register
		pGPIOHandle->pGPIOx->OTYPER |= temp;	// set register

		temp = 0;
	}

	// 5. Configure the alternate functionality
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

// Peripheral clock setup
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

// Data read / write
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber) {

	uint8_t value;
	value = (uint8_t)( (pGPIOx->IDR >> pinNumber) & 0x00000001 );
	return value;

}

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx) {

	uint16_t value;
	value = (uint16_t)(pGPIOx->IDR);
	return value;

}

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

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value) {

	pGPIOx->ODR = Value;

}

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber) {

	pGPIOx->ODR ^= ( 1 << pinNumber );

}

// IRQ configuration & ISR handling
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

void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority) {

	// 1. Determine the proper IPR register
	uint8_t IPRx = IRQNumber / 4;
	uint8_t IPRxSection = IRQNumber % 4;

	uint8_t shiftAmount = (8 * IPRxSection) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + IPRx) |= (IRQPriority << shiftAmount);

}

void GPIO_IRQHandling(uint8_t PinNumber) {

	// Check for pended EXTI PR register bit corresponding to the pin number
	if(EXTI->PR & ( 1 << PinNumber )) {
		// Clear the pended bit
		EXTI->PR |= ( 1 << PinNumber );
	}

}
