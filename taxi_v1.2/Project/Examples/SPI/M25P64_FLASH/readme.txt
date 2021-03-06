/**
  @page SPI_M25P64_FLASH SPI_M25P64_FLASH
  
  @verbatim
  ******************** (C) COPYRIGHT 2009 STMicroelectronics *******************
  * @file SPI/M25P64_FLASH/readme.txt 
  * @author  MCD Application Team
  * @version  V3.0.0
  * @date  04/06/2009
  * @brief  Description of the SPI M25P64_Flash Example.
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

This example provides a basic example of how to use the SPI firmware library
and an associate SPI FLASH driver to communicate with an M25P64 FLASH.

The first step consist in reading the SPI Flash ID. A comparison between the ID 
read from SPI flash and the expected one is done and a specific GPIO pin is set 
in case of success otherwise this GPIO pin is reset.  

Using this driver the program performs an erase of the sector to be accessed, a 
write of a Tx_Buffer, defined in the main.c file, to the memory followed by a read
of the written data. Then data read from the memory stored in the Rx_Buffer are
compared with the expected values of the Tx_Buffer. The result of this comparison
is stored in the "TransferStatus1" variable.

A second erase of the same sector is done at the end, and a test is done to be
sure that all the data written there are erased further to the sector erase. All
the data location are read and checked with 0xFF value. The result of this test
is stored in "TransferStatus2" variable which is FAILED in case of error.

The SPI1 is configured as Master with an 8bits data size. A GPIO pin is used
as output push-pull to drive the SPI Flash chip select pin.  
The FLASH_WriteAddress and the FLASH_ReadAddress where the program start the write 
and the read operations are defined in the main.c file. 
The system clock is set to 72MHz and SPI1 baudrate to 18 Mbit/s.

@par Directory contents 

  - SPI/M25P64_FLASH/platform_config.h    Evaluation board specific configuration file
  - SPI/M25P64_FLASH/stm32f10x_conf.h     Library Configuration file
  - SPI/M25P64_FLASH/stm32f10x_it.c       Interrupt handlers
  - SPI/M25P64_FLASH/stm32f10x_it.h       Header for stm32f10x_it.c
  - SPI/M25P64_FLASH/main.c               Main program
  - SPI/M25P64_FLASH/spi_flash.c          SPI FLASH driver
  - SPI/M25P64_FLASH/spi_flash.h          Header for the spi_flash.c file

@par Hardware and Software environment 

  - This example runs on STM32F10x High-Density, STM32F10x Medium-Density and
    STM32F10x Low-Density Devices.
  
  - This example has been tested with STMicroelectronics STM3210E-EVAL (STM32F10x 
    High-Density) and STM3210B-EVAL (STM32F10x Medium-Density) evaluation boards 
    and can be easily tailored to any other supported device and development 
    board.
    To select the STMicroelectronics evaluation board used to run the example, 
    uncomment the corresponding line in platform_config.h and spi_flash.h files.
    
  - STM3210E-EVAL Set-up 
    - Use LD1 and LD2 leds connected respectively to PF.06 and PF.07 pins

@note in STM3210E-EVAL board, the jumper 14 (USB Disconnect) must be set in 
      position 1<->2 in order to not interfer with SPI2 MISO pin PB14.
      
  - STM3210B-EVAL Set-up  
    - Use LD1 and LD2 leds connected respectively to PC.06 and PC.07 pins
 
On the STMicroelectronics STM3210B-EVAL and STM3210E-EVAL evaluation boards, 
this SPI Flash is already available and there is no need to any extra hardware 
connections. 

  - Different platform Set-up 
    - Connect both SPI1 and SPI FLASH pins as following:
      - Connect SPI1_NSS (PA.04) pin to SPI Flash chip select (pin1) and use the 
        STM3210B-EVAL hardware configuration defines.
      - Connect SPI1_SCLK (PA.05) pin to SPI Flash serial clock (pin6).
      - Connect SPI1_MISO (PA.06) pin to SPI Flash serial data output (pin2).
      - Connect SPI1_MOSI (PA.07) pin to SPI Flash serial data input (pin5).
      - Connect SPI Flash Write Protect (pin3) to Vdd
      - Connect SPI Flash Hold (pin7) to Vdd
      - Connect SPI Flash Vcc (pin8) to Vdd
      - Connect SPI Flash Vss (pin4) to Vss

@par How to use it ? 

In order to make the program work, you must do the following :
- Create a project and setup all project configuration
- Add the required Library files :
  - stm32f10x_gpio.c  
  - stm32f10x_rcc.c 
  - stm32f10x_spi.c
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
