STM32F407 USART Transmission Using DMA (Bare-Metal)

Overview

This project demonstrates UART data transmission using DMA with interrupt handling on the STM32F407 microcontroller using pure bare-metal register programming without HAL or LL drivers.

Transmission is triggered by an external interrupt (user button) and handled by DMA1 Stream6.  
A DMA Full Transfer callback is implemented to reload the DMA stream and allow repeated transmissions.

The project is developed and built using STM32CubeIDE.


Features

- Bare-metal peripheral configuration  
- USART2 configured in transmit-only mode at 115200 baud  
- DMA1 Stream6 configured for memory to peripheral transfer  
- External interrupt (EXTI0) triggers USART DMA transmission  
- DMA Full Transfer interrupt with callback-based stream reinitialization  
- Reusable DMA stream for repeated UART transmissions  


Repository Structure

stm32f407-usart-dma/
|
|-- README.md
|-- src/
|   |-- main.c
|   |-- gpio_driver.c
|   |-- usart_driver.c
|   |-- dma_driver.c
|   |-- stm32f407.h
|
|-- inc/
|   |-- gpio_driver.h
|   |-- usart_driver.h
|   |-- dma_driver.h
|
|-- startup/
|   |-- startup_stm32f407xx.s
|
|-- linker/
|   |-- stm32f407.ld


Hardware Configuration

Board  
STM32F407 Discovery  

Pin Mapping  

USART2_TX    PA2    UART transmit  
USART2_RX    PA3    UART receive (configured but unused)  
User Button  PA0    External interrupt trigger  


DMA Configuration

Peripheral      USART2_TX  
DMA Controller  DMA1  
Stream          Stream6  
Channel         Channel 4  
Direction       Memory to Peripheral  


System Operation

1. GPIO, USART2, DMA1 Stream6 and NVIC are initialized  
2. User presses the button on PA0 and EXTI0 interrupt occurs  
3. USART DMA mode is enabled inside the EXTI interrupt handler  
4. DMA transfers the message buffer to the USART data register  
5. DMA Full Transfer interrupt is generated  
6. Full Transfer callback reloads NDTR, disables USART DMA mode and re-enables the DMA stream  

This allows repeated UART transmissions on every button press.


Interrupt Handling

EXTI0_IRQHandler  
Enables USART DMA mode on external interrupt  

DMA1_Stream6_IRQHandler  
Handles DMA Full Transfer interrupt  


Callback Implemented

FT_complete_callback  

- Reloads NDTR with buffer length  
- Disables USART DMA mode  
- Re-enables DMA stream  


Transferred Message

"UART Tx testing...\n\r"


Development Environment

IDE             STM32CubeIDE  
Microcontroller STM32F407  
Toolchain       GNU Arm Embedded Toolchain (arm-none-eabi-gcc)  
Debugger        ST-Link  

Terminal Settings  

Baud rate   115200  
Data bits   8  
Stop bits   1  
Parity      None  

