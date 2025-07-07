# Simple HAL inplementation for the STM32F411XX
This HAL (Hardware Abstraction Layer) is a simple, yet robust driver implementation of the STM32F411XX. It has extensive details documented on the code and it's mostly used on builds that have limited space. It contrasts the available ST HAL for the STM32F411XX with cleaner functions, simple implementations.

## How to Use it
Follow the guideline on the example file for each inmplementation. There are example implementation files for major communication protocols (I2C, SPI and USART) at the `examples/` folder. The only major necessity is to define an external clock at `include/drivers/stm32f411xx_clock.h` if you are using a HSE. Since there are logic calculations based on the clock (ex. the I2C_CCR register use the value of the clock to determine the correct pattern for I2C communication), it's important to have it defined first thing.

All implementations have a handler structure and embedded documentation that helps you with a high level guide on how to setup the necessary configurations. All main communication protocols have init, read and write functions, as well as interrupt classes.

## How to Colaborate?
Feel free to test, report issues and add your implementations. All PRs will be reviewed and merged accordingly.

## License
Licensed under the Open Software License version 3.0

