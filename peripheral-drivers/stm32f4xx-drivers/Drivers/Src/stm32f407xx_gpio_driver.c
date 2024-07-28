
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

// Init & De-Init
void GPIO_Init(GPIO_Handle_t *pGPIOHandle) {

	uint32_t temp = 0;

	// 1. Configure mode of GPIO pin
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG) {
		temp = ( pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
		pGPIOHandle->pGPIOx->MODER &= ~( 0x03 << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber ) );	// clear register
		pGPIOHandle->pGPIOx->MODER |= temp;	// set register
	}
	else {

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
	temp = ( pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
	pGPIOHandle->pGPIOx->OTYPER &= ~( 0x01 << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber ) );		// clear register
	pGPIOHandle->pGPIOx->OTYPER |= temp;	// set register

	temp = 0;

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
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t En) {

}

void GPIO_IRQHandling(uint8_t PinNumber) {

}
