/**
  @page USART_HyperTerminal_Interrupt USART_HyperTerminal_Interrupt
  
  @verbatim
  ******************** (C) COPYRIGHT 2009 STMicroelectronics *******************
  * @file USART/HyperTerminal_Interrupt/readme.txt 
  * @author  MCD Application Team
  * @version  V3.0.0
  * @date  04/06/2009
  * @brief  Description of the USART Hyperterminal interrupt Example.
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

This example provides a description of how to use the USART1 Transmit and Receive 
interrupts to communicate with the hyperterminal.

First, the USART1 sends the TxBuffer to the hyperterminal and still waiting for
a string from the hyperterminal that you must enter.
The string that you have entered is stored in the RxBuffer array. The receive
buffer have a RxBufferSize bytes as maximum (the reception is stopped when this
maximum receive value is reached).

The USART1 is configured as follow:
    - BaudRate = 9600 baud  
    - Word Length = 8 Bits (7 data bit + 1 parity bit)
    - Two Stop Bit
    - Odd parity
    - Hardware flow control disabled (RTS and CTS signals)
    - Receive and transmit enabled

@note When the parity is enabled, the computed parity is inserted at the MSB
position of the transmitted data.
  
@par Directory contents 

  - USART/HyperTerminal_Interrupt/stm32f10x_conf.h  Library Configuration file
  - USART/HyperTerminal_Interrupt/stm32f10x_it.h    Interrupt handlers header file
  - USART/HyperTerminal_Interrupt/stm32f10x_it.c    Interrupt handlers source file
  - USART/HyperTerminal_Interrupt/main.c            Main program

@par Hardware and Software environment 

  - This example runs on STM32F10x High-Density, STM32F10x Medium-Density and
    STM32F10x Low-Density Devices.
  
  - This example has been tested with STMicroelectronics STM3210E-EVAL (STM32F10x 
    High-Density) and STM3210B-EVAL (STM32F10x Medium-Density) evaluation boards 
    and can be easily tailored to any other supported device and development 
    board.

  - STM3210E-EVAL Set-up 
    - The USART1 signals (Rx and Tx) must be connected to a DB9 connector using
      a RS232 transceiver.

    - Connect a null-modem female/female RS232 cable between the DB9 connector 
     (CN12 on STM3210E-EVAL board) and PC serial port.  

  - STM3210B-EVAL Set-up 
    - The USART1 signals (Rx and Tx) must be connected to a DB9 connector using
      a RS232 transceiver.

    - Connect a null-modem female/female RS232 cable between the DB9 connector 
     (CN6 on STM3210B-EVAL board) and PC serial port.  
     
  - Hyperterminal configuration:
    - Word Length = 7 Bits
    - Two Stop Bit
    - Odd parity
    - BaudRate = 9600 baud
    - flow control: None 
      
@par How to use it ? 

In order to make the program work, you must do the following :
- Create a project and setup all project configuration
- Add the required Library files :
  - stm32f10x_gpio.c 
  - stm32f10x_rcc.c
  - stm32f10x_usart.c 
  - misc.c 
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
