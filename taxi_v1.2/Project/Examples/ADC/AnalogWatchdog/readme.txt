/**
  @page ADC_AnalogWatchdog ADC_AnalogWatchdog
  
  @verbatim
  ******************** (C) COPYRIGHT 2009 STMicroelectronics *******************
  * @file ADC/AnalogWatchdog/readme.txt 
  * @author  MCD Application Team
  * @version  V3.0.0
  * @date  04/06/2009
  * @brief  Description of the ADC analog watchdog example.
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

This example describes how to use the ADC analog watchdog to guard continuously  
an ADC channel.
The ADC1 is configured to convert continuously ADC channel14. The analog watchdog
is configured and enabled to guard a single regular channel. 
Each time the channel14 converted value exceed programmed analog watchdog high 
threshold (value 0x0B00) or goes down analog watchdog low threshold (value 0x0300)
an AWD interrupt is generated and the output pin connected to LED1 is toggled. The
LED will bright as long as the AWD interrupt is generated which means that the 
converted value of regular ADC channel14 is outside the range limited by high and 
low analog watchdog thresholds.
The ADC1 clock is set to 14 MHz.

@par Directory contents 

  - ADC/AnalogWatchdog/platform_config.h  Evaluation board specific configuration file
  - ADC/AnalogWatchdog/stm32f10x_conf.h   Library Configuration file
  - ADC/AnalogWatchdog/stm32f10x_it.c     Interrupt handlers
  - ADC/AnalogWatchdog/stm32f10x_it.h     Interrupt handlers header file
  - ADC/AnalogWatchdog/main.c             Main program

@par Hardware and Software environment 

  - This example runs on STM32F10x High-Density, STM32F10x Medium-Density and
    STM32F10x Low-Density Devices.
  
  - This example has been tested with STMicroelectronics STM3210E-EVAL (STM32F10x 
    High-Density) and STM3210B-EVAL (STM32F10x Medium-Density) evaluation boards 
    and can be easily tailored to any other supported device and development 
    board. 
    To select the STMicroelectronics evaluation board used to run the example, 
    uncomment the corresponding line in platform_config.h file.

  - STM3210E-EVAL Set-up 
    - Use LD1 led connected to PF.06 pin
    - Connect a variable power supply 0-3.3V to ADC Channel14 mapped on pin PC.04
      (potentiometer RV1)     

 - STM3210B-EVAL  
    - Use LD1 led connected to PC.06 pin
    - Connect a variable power supply 0-3.3V to ADC Channel14 mapped on pin PC.04
      (potentiometer RV1) 
         
@par How to use it ? 

In order to make the program work, you must do the following :
- Create a project and setup all project configuration
- Add the required Library files :
  - stm32f10x_adc.c 
  - stm32f10x_gpio.c
  - stm32f10x_flash.c 
  - stm32f10x_rcc.c 
  - misc.c   
      
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
