
/*=====================================================================================*\
| Author:   Christopher Coyne                                          July 26th, 2024  |
| --------------------------------------------------------------------------------------|
| Date:     July 26th, 2024                                                             |
| --------------------------------------------------------------------------------------|
| MODULE:     [ GPIO driver ]                                                           |
| FILE:       stm32f407xx_gpio_driver.h                                                 |
| --------------------------------------------------------------------------------------|
| DESCRIPTION:                                                                          |
|    ...  		                                                                        |
\*=====================================================================================*/

#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_

/****************************************************
 *  Include files                                   *
 ****************************************************/

#include "stm32f407xx.h"

/****************************************************
 *  Structures									    *
 ****************************************************/

typedef struct {
	uint8_t GPIO_PinNumber;				// Possible values from @GPIO_PIN_NUMBERS
	uint8_t GPIO_PinMode;				// Possible values from @GPIO_PIN_MODES
	uint8_t GPIO_PinSpeed;				// Possible values from @GPIO_PIN_SPEEDS
	uint8_t GPIO_PinPuPdControl;		// Possible values from @GPIO_PIN_PUPD_CONTROL
	uint8_t GPIO_PinOPType;				// Possible values from @GPIO_PIN_OUTPUT_TYPES
	uint8_t GPIO_PinAltFunMode;
} GPIO_PinConfig_t;

typedef struct {
	GPIO_RegDef_t *pGPIOx;				// pointer to hold the base adddress of the GPIO port
	GPIO_PinConfig_t GPIO_PinConfig;	// holds GPIO pin configuration settings
} GPIO_Handle_t;

/****************************************************
 *  Macros										    *
 ****************************************************/

// @GPIO_PIN_NUMBERS
// GPIO pins
#define GPIO_PIN_0				0		// GPIO pin 0
#define GPIO_PIN_1				1		// GPIO pin 1
#define GPIO_PIN_2				2		// GPIO pin 2
#define GPIO_PIN_3				3		// GPIO pin 3
#define GPIO_PIN_4				4		// GPIO pin 4
#define GPIO_PIN_5				5		// GPIO pin 5
#define GPIO_PIN_6				6		// GPIO pin 6
#define GPIO_PIN_7				7		// GPIO pin 7
#define GPIO_PIN_8				8		// GPIO pin 8
#define GPIO_PIN_9				9		// GPIO pin 9
#define GPIO_PIN_10				10		// GPIO pin 10
#define GPIO_PIN_11				11		// GPIO pin 11
#define GPIO_PIN_12				12		// GPIO pin 12
#define GPIO_PIN_13				13		// GPIO pin 13
#define GPIO_PIN_14				14		// GPIO pin 14
#define GPIO_PIN_15				15		// GPIO pin 15

// @GPIO_PIN_MODES
// GPIO non-interrupt modes
#define GPIO_MODE_IN			0		// Input mode
#define GPIO_MODE_OUT			1		// Output mode
#define GPIO_MODE_ALTFN			2 		// Alternate function mode
#define GPIO_MODE_ANALOG		3		// Analog mode
// GPIO interrupt modes
#define GPIO_MODE_INT_FT		4		// Input falling edge trigger
#define GPIO_MODE_INT_RT		5		// Input rising edge trigger
#define GPIO_MODE_INT_RFT		6		// Input rising / falling edge trigger

// @GPIO_PIN_OUTPUT_TYPES
// Output types
#define GPIO_OP_TYPE_PP			0		// Output push-pull
#define GPIO_OP_TYPE_OD			1		// Output open drain

// @GPIO_PIN_SPEEDS
// Speeds
#define GPIO_SPEED_LOW			0		// Low speed
#define GPIO_SPEED_MEDIUM		1		// Medium speed
#define GPIO_SPEED_FAST			2		// Fast speed
#define GPIO_SPEED_HIGH			3		// High speed

// @GPIO_PIN_PUPD_CONTROL
// Pull-up / pull-down
#define GPIO_NO_PUPD			0		// No pull-up / pull-down resistor enabled
#define GPIO_PIN_PU				1		// Pull-up resistor enabled
#define GPIO_PIN_PD				2		// Pull-down resistor enabled

/****************************************************
 *  API prototypes								    *
 ****************************************************/

// Init & De-Init
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

// Peripheral clock setup
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t En);

// Data read / write
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber);

// IRQ configuration & ISR handling
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t En);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);


#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
