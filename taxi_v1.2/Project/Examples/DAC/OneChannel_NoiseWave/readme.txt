/**
  @page DAC_OneChannel_NoiseWave DAC_OneChannel_NoiseWave
  
  @verbatim
  ******************** (C) COPYRIGHT 2009 STMicroelectronics *******************
  * @file DAC/OneChannel_NoiseWave/readme.txt 
  * @author  MCD Application Team
  * @version  V3.0.0
  * @date  04/06/2009
  * @brief  Description of the DAC one channel noise wave example.
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

This example describes how to use one DAC channel to generate a signal with noise
waves on DAC Channel1 output.

DAC channel1 conversion are configured to be triggered by software with noise wave 
generation.12bit left data alignement is selected since we choose to acces DAC_DHR12L1
register. Bits 0 to 8 are masked for the Linear feedback shift register. 
DAC channel1 is then enabled. DAC Channel1 DHR12L1 register is configured to have
an output voltage of VREF/2.

Software triggers are generated continuously in an infinite loop, and on each
trigger the DAC channel1 start the conversion and calculate the noise value to
apply on the DAC channel1 output.

The output signal with noise waves can be visualized by connecting PA.04 pin to
an oscilloscope.

@par Directory contents 

  - DAC/OneChannel_NoiseWave/stm32f10x_conf.h     Library Configuration file
  - DAC/OneChannel_NoiseWave/stm32f10x_it.c       Interrupt handlers
  - DAC/OneChannel_NoiseWave/stm32f10x_it.h       Header for stm32f10x_it.c
  - DAC/OneChannel_NoiseWave/main.c               Main program

@par Hardware and Software environment 

  - This example runs only on STM32F10x High-Density Devices.
  
  - This example has been tested with STMicroelectronics STM3210E-EVAL (STM32F10x 
    High-Density) evaluation board and can be easily tailored to any other 
    supported device and development board.

  - STM3210E-EVAL Set-up 
    - Connect PA.04 pin to an oscilloscope
    
@par How to use it ? 

In order to make the program work, you must do the following:
- Create a project and setup all project configuration
- Add the required Library files:
  - stm32f10x_gpio.c
  - stm32f10x_dac.c  
  - stm32f10x_rcc.c
  - system_stm32f10x.c   
      
- Edit stm32f10x.h file to select the device you are working on (#define 
  STM32F10X_HD, in this case).
  
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
