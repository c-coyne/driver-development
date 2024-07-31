# Embedded Systems Driver Library

This repository showcases a collection of low-level drivers for embedded systems, including peripheral, communication, and device drivers. Each driver is implemented in C and organized into directories by type, with individual READMEs for detailed documentation and usage.

Current contents of the repo include:
- [**GPIO driver for STM32F407VG MCU**](peripheral-drivers/stm32f4xx-drivers/Doc/GPIO_README.md) 

    (`driver-development/peripheral-drivers/stm32f4xx-drivers/Drivers/Src/stm32f407xx_gpio_driver.c`)

## Table of Contents

- [Overview](#overview)
- [Repository Structure](#repository-structure)
- [Getting Started](#getting-started)
- [License](#license)
- [Contact](#contact)
- [Acknowledgements](#acknowledgements)

## Overview

### STM32F4xx GPIO Driver
The GPIO driver provides an interface to initialize, configure, and control the GPIO pins of the ARM Cortex-M4 STM32F407xx family of MCU's. It supports various configurations such as input, output, analog, and interrupt modes, as well as pin speed, pull-up/pull-down settings, and alternate functions.

## Repository Structure

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

## Getting Started

To get started with the drivers in this repository:

1. **Clone the repository:**
    ```bash
    git clone https://github.com/c-coyne/driver-development.git
    ```

2.  **Navigate to the desired driver directory:**
    ```bash
    cd driver-development/peripheral-drivers/stm32f4xx-drivers
    ```

3. **Follow the instructions in the driver README.md file for further setup and usage details.**

## License

This project is licensed under the [MIT License](https://opensource.org/license/MIT).

## Contact

Christopher Coyne: christopher.w.coyne@gmail.com  
Project Link: https://github.com/c-coyne/driver-development

## Acknowledgements

**STM32CubeIDE:** STMicroelectronics

**FastBit Embedded Brain Academy:** Parts of this project constitute a modified and extended version of one of the many projects in their [_Mastering Microcontrollers and Embedded Driver Development_](https://www.udemy.com/course/mastering-microcontroller-with-peripheral-driver-development/) course.