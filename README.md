# PlayStation2 Controller driver (asynchronous transport layer for STM32F407)

This repository contains an implementation of the transport layer for the [PlayStation2 Controller asynchronous driver](https://github.com/godunko/a0b-playstation2_controller-async/tree/main) for the STM32F407 MCU family. It uses SPI2, TIM7, EXTI11 peripherals. Implementation assumes that processor's clock run at 168 MHz.

| PlayStation2 Controller | STM32F407 |
| --- | --- |
| 1 (Data)        | PB14 (MISO) |
| 2 (Command)     | PB15 (MOSI) |
| 6 (Attention)   | PB12 (NSS)  |
| 7 (Clock)       | PB13 (SCK)  |
| 9 (Acknowledge) | PB11 (ACK)  |

Note, all connections are mandatory (including Acknowledge signal).
