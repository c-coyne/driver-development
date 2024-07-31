# GPIO Driver for STM32F407xx MCU

This repository contains a General Purpose Input/Output (GPIO) driver implemented in C for the ARM Cortex-M4 STM32F407xx family of MCU's.

## Table of Contents

- [Overview](#overview)
- [Features](#features)
- [File Structure](#file-structure)
- [Usage](#usage)
- [API Documentation](#api-documentation)

## Overview

The GPIO driver provides an interface to initialize, configure, and control the GPIO pins of the STM32F407xx MCU. It supports various configurations such as input, output, analog, and interrupt modes, as well as pin speed, pull-up/pull-down settings, and alternate functions.

## Features

- Initialize and de-initialize GPIO pins
- Configure pin modes, speeds, and pull-up/pull-down settings
- Set and clear output pins
- Read from input pins and ports
- Configure and handle GPIO interrupts
- Control the peripheral clock for GPIO ports
- Lock pins to prevent unintended reconfiguration

## File Structure

```
driver-development/
├── peripheral-drivers/
│ ├── stm32f4xx-drivers/
│ │ ├── Doc/
| | | └── GPIO_README.md
│ │ ├── Drivers/
| | | ├── Inc/
| | | | ├── stm32f407xx_gpio_driver.h
| | | | └── stm32f407xx.h
| | | └── Src/
| | | | └── stm32f407xx_gpio_driver.c
│ │ ├── Inc/
│ │ ├── Src/
| | | ├── main.c
| | | ├── syscalls.c
| | | └── sysmem.c
│ │ └── Startup/
| | | └── startup_stm32f407vgtx.s
├── LICENSE
└── README.md

```

## Usage

1. Clone the repository:
   ```bash
   git clone https://github.com/c-coyne/driver-development.git
   cd driver-development
   ```

2. Include the driver in your project:
   ```c
   #include "stm32f407xx_gpio_driver.h"
   ```

3. Initialize and configure a GPIO pin:
   ```c
   GPIO_Handle_t gpioHandle;
   gpioHandle.pGPIOx = GPIOA;
   gpioHandle.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
   gpioHandle.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT;
   gpioHandle.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
   gpioHandle.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
   gpioHandle.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

   GPIO_Init(&gpioHandle);
   ```

4. Set or reset the GPIO pin:
   ```c
   GPIO_WriteToOutputPin(GPIOA, GPIO_PIN_NO_5, GPIO_PIN_SET);
   GPIO_WriteToOutputPin(GPIOA, GPIO_PIN_NO_5, GPIO_PIN_RESET);
   ```

## API Documentation

### `GPIO_Init`

Initializes the specified GPIO pin(s) according to the provided configuration.

```c
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
```

#### Parameters
- `pGPIOHandle`: Pointer to the GPIO handle structure containing configuration settings for the pin.

#### Example
Initialize a GPIO pin as an output with no pull-up or pull-down resistors and high-speed frequency.

```c
GPIO_Handle_t GpioLed;

GpioLed.pGPIOx = GPIOD;
GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;
GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

GPIO_PeriClockControl(GPIOD, ENABLE);

GPIO_Init(&GpioLed);
```

### `GPIO_DeInit`

Resets the specified GPIO port.

```c
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);
```

#### Parameters
- `pGPIOx`: Pointer to the GPIO port base address to be de-initialized.

#### Example
De-initialize GPIO port A to reset its configuration.

```c
GPIO_DeInit(GPIOA);
```

### `GPIO_PeriClockControl`

Enables or disables a peripheral clock.

```c
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t En);
```

#### Parameters
- `pGPIOx`: Pointer to the GPIO port base address.
- `En`: Enable or disable command for the peripheral clock. Use ENABLE to turn on the clock and DISABLE to turn off the clock.

#### Example
Enable the GPIO port D peripheral clock.

```c
GPIO_PeriClockControl(GPIOD, ENABLE);
```

### `GPIO_ReadFromInputPin`

Reads the state of the specified GPIO pin.

```c
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber);
```

#### Parameters
- `pGPIOx`: Pointer to the GPIO port base address.
- `pinNumber`: Pin number from which the value is to be read.

#### Returns
- Value read from the specified GPIO input pin (`0` or `1`).

#### Example
Read the state of a specific GPIO pin.

```c
uint8_t pinState = GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_5);
```

### `GPIO_ReadFromInputPort`

Reads the state of the specified GPIO port.

```c
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
```

#### Parameters
- `pGPIOx`: Pointer to the GPIO port base address.

#### Returns
Value read from the input data register of the specified GPIO port.

#### Example
Read the state of GPIO port D.

```c
uint16_t portState = GPIO_ReadFromInputPort(GPIOD);
```

### `GPIO_WriteToOutputPin`

Sets or clears the specified GPIO pin.

```c
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber, uint8_t Value);
```

#### Parameters
- `pGPIOx`: Pointer to the GPIO port base address.
- `pinNumber`: Pin number to which the value is to be written.
- `Value`: Value to be written to the pin (`GPIO_PIN_SET` or `GPIO_PIN_RESET`).

#### Example
Set a specific GPIO pin to high or low.

```c
GPIO_WriteToOutputPin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
```

### `GPIO_WriteToOutputPort`

Sets or clears the specified GPIO port.

```c
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
```

#### Parameters
- `pGPIOx`: Pointer to the GPIO port base address.
- `Value`: 16-bit value to be written to the output data register.

#### Example
Set a specific GPIO port to high or low.

```c
uint16_t value = 15;
GPIO_WriteToOutputPort(GPIOA, value);
```

### `GPIO_ToggleOutputPin`

Toggles the state of the specified GPIO pin.

```c
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber);
```

#### Parameters
- `pGPIOx`: Pointer to the GPIO port base address.
- `PinNumber`: GPIO pin to be toggled.

#### Example
Toggle the state of a specific GPIO pin.

```c
GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_5);
```

### `GPIO_IRQInterruptConfig`

Configures the interrupt for a given IRQ number.

```c
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t En);
```

#### Parameters
- `IRQNumber`: IRQ number to be configured.
- `En`: Enable or disable command for the IRQ (`ENABLE` to enable the interrupt, `DISABLE` to disable the interrupt).

#### Example
Configure an interrupt for IRQ number 4.

```c
GPIO_IRQInterruptConfig(4, ENABLE);
```

### `GPIO_IRQPriorityConfig`

Configures the priority of an interrupt for a given IRQ number.

```c
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
```

#### Parameters
- `IRQNumber`: IRQ number to be configured.
- `IRQPriority`: Priority level to be set for the IRQ number.

#### Example
Configure the interrupt priority for IRQ number 4.

### `GPIO_IRQHandling`

Handles the IRQ for a specified GPIO pin by clearing the pending bit.

```c
void GPIO_IRQHandling(uint8_t PinNumber);
```

#### Parameters
- `PinNumber`: GPIO pin number for which the interrupt handling is to be performed.

#### Example
Configure the interrupt priority for IRQ number 4.

```c
GPIO_IRQHandling(GPIO_PIN_15);
```

### `GPIO_LockPins`

Locks the configuration of the specified GPIO pins.

```c
uint8_t GPIO_LockPins(GPIO_TypeDef *GPIOx, uint16_t PinNumbers);
```

#### Parameters
- `GPIOx`: Pointer to the GPIO port base address.
- `PinNumbers`: GPIO pin numbers to be locked.

#### Returns
- Lock status (`0` for unlocked or `1` for locked).

#### Example
Lock the configuration of specific GPIO pins to prevent changes. In this case, pins A5 and A6 are locked.

```c
uint8_t lockStatus = GPIO_LockPins(GPIOA, ( 1 << 5 ) | ( 1 << 6 ));
```