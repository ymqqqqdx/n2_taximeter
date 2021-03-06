/**
  @page USART_Synchronous USART_Synchronous
  
  @verbatim
  ******************** (C) COPYRIGHT 2009 STMicroelectronics *******************
  * @file USART/Synchronous/readme.txt 
  * @author  MCD Application Team
  * @version  V3.0.0
  * @date  04/06/2009
  * @brief  Description of the USART Synchronous Example.
  ******************************************************************************
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  ******************************************************************************
   @endverbatim

@par Example Description 

This example provides a basic communication between USART1 (Synchronous mode) 
and SPI1 using flags.

First, the USART1 sends data from TxBuffer1 buffer to SPI1 using USART1 TXE flag.
Data received, using RXNE flag, by SPI1 is stored in RxBuffer2 then compared with
the sent ones and the result of this comparison is stored in the "TransferStatus1" 
variable.
 
Then, the SPI1 sends data from TxBuffer2 buffer to USART1 using SPI1 TXE flag.
Data received, using RXNE flag, by USART1 is stored in RxBuffer1 then compared with
the sent ones and the result of this comparison is stored in the "TransferStatus2" 
variable.

USART1 configured as follow:
  - BaudRate = 115200 baud  
  - Word Length = 8 Bits
  - One Stop Bit
  - No parity
  - Hardware flow control disabled (RTS and CTS signals)
  - Receive and transmit enabled
  - USART Clock enabled
  - USART CPOL: Clock is active high
  - USART CPHA: Data is captured on the second edge 
  - USART LastBit: The clock pulse of the last data bit is output to the SCLK pin

SPI1 configured as follow:
  - Direction = 2 Lines FullDuplex
  - Mode = Slave Mode
  - DataSize = 8 Bits
  - CPOL = Clock is active high
  - CPHA = Data is captured on the second edge 
  - NSS = NSS Software
  - First Bit = First Bit is the LSB  

@par Directory contents 

  - USART/Synchronous/stm32f10x_conf.h  Library Configuration file
  - USART/Synchronous/stm32f10x_it.h    Interrupt handlers header file
  - USART/Synchronous/stm32f10x_it.c    Interrupt handlers
  - USART/Synchronous/main.c            Main program

@par Hardware and Software environment 

  - This example runs on STM32F10x High-Density, STM32F10x Medium-Density and
    STM32F10x Low-Density Devices.
  
  - This example has been tested with STMicroelectronics STM3210E-EVAL (STM32F10x 
    High-Density) and STM3210B-EVAL (STM32F10x Medium-Density) evaluation boards 
    and can be easily tailored to any other supported device and development 
    board.

  - STM3210E-EVAL Set-up 
    - Connect USART1_Tx(PA.09) to SPI1_MOSI(PA.07), USART1_Rx(PA.10) to 
      SPI1_MISO(PA.06) and USART1_CK(PA.08) to SPI1_SCK(PA.05).

  - STM3210B-EVAL Set-up 
    - Connect USART1_Tx(PA.09) to SPI1_MOSI(PA.07), USART1_Rx(PA.10) to 
      SPI1_MISO(PA.06) and USART1_CK(PA.08) to SPI1_SCK(PA.05).        
  
@par How to use it ? 

In order to make the program work, you must do the following :
- Create a project and setup all project configuration
- Add the required Library files :
  - stm32f10x_gpio.c 
  - stm32f10x_rcc.c 
  - stm32f10x_spi.c 
  - stm32f10x_usart.c 
  - system_stm32f10x.c 
     
- Edit stm32f10x.h file to select the device you are working on.
  
@b Tip: You can tailor the provided project template to run this example, for 
        more details please refer to "stm32f10x_stdperiph_lib_um.chm" user 
        manual; select "Peripheral Examples" then follow the instructions 
        provided in "How to proceed" section.   
- Link all compiled files and load your image into target memory
- Run the example

@note
 - Low-density devices are STM32F101xx and STM32F103xx microcontrollers where
   the Flash memory density ranges between 16 and 32 Kbytes.
 - Medium-density devices are STM32F101xx and STM32F103xx microcontrollers where
   the Flash memory density ranges between 32 and 128 Kbytes.
 - High-density devices are STM32F101xx and STM32F103xx microcontrollers where
   the Flash memory density ranges between 256 and 512 Kbytes.
   
 * <h3><center>&copy; COPYRIGHT 2009 STMicroelectronics</center></h3>
 */
