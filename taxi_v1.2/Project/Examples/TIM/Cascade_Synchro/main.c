/**
  ******************************************************************************
  * @file TIM/Cascade_Synchro/main.c 
  * @author  MCD Application Team
  * @version  V3.0.0
  * @date  04/06/2009
  * @brief  Main program body
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2009 STMicroelectronics</center></h2>
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"

/** @addtogroup StdPeriph_Examples
  * @{
  */

/** @addtogroup TIM_Cascade_Synchro
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
TIM_OCInitTypeDef  TIM_OCInitStructure;

/* Private function prototypes -----------------------------------------------*/
void RCC_Configuration(void);
void GPIO_Configuration(void);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval : None
  */
int main(void)
{
  /* System Clocks Configuration */
  RCC_Configuration();

  /* GPIO Configuration */
  GPIO_Configuration();

  /* Timers synchronisation in cascade mode ----------------------------
     1/TIM2 is configured as Master Timer:
     - PWM Mode is used
     - The TIM2 Update event is used as Trigger Output  

     2/TIM3 is slave for TIM2 and Master for TIM4,
     - PWM Mode is used
     - The ITR1(TIM2) is used as input trigger 
     - Gated mode is used, so start and stop of slave counter
      are controlled by the Master trigger output signal(TIM2 update event).
      - The TIM3 Update event is used as Trigger Output. 

      3/TIM4 is slave for TIM3,
     - PWM Mode is used
     - The ITR2(TIM3) is used as input trigger
     - Gated mode is used, so start and stop of slave counter
      are controlled by the Master trigger output signal(TIM3 update event).

     The TIMxCLK is fixed to 72 MHz, the TIM2 counter clock is 72 MHz.

     The Master Timer TIM2 is running at TIM2 frequency :
     TIM2 frequency = (TIM2 counter clock)/ (TIM2 period + 1) = 281.250 KHz 
     and the duty cycle = TIM2_CCR1/(TIM2_ARR + 1) = 25%.

     The TIM3 is running:
     - At (TIM2 frequency)/ (TIM3 period + 1) = 70.312 KHz and a duty cycle
     equal to TIM3_CCR1/(TIM3_ARR + 1) = 25%

     The TIM4 is running:
     - At (TIM3 frequency)/ (TIM4 period + 1) = 17.578 KHz and a duty cycle
     equal to TIM4_CCR1/(TIM4_ARR + 1) = 25%
  -------------------------------------------------------------------- */

  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = 255;
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

  TIM_TimeBaseStructure.TIM_Period = 3;
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

  TIM_TimeBaseStructure.TIM_Period = 3;
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

  /* Master Configuration in PWM1 Mode */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 64;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

  TIM_OC1Init(TIM2, &TIM_OCInitStructure);

  /* Select the Master Slave Mode */
  TIM_SelectMasterSlaveMode(TIM2, TIM_MasterSlaveMode_Enable);

  /* Master Mode selection */
  TIM_SelectOutputTrigger(TIM2, TIM_TRGOSource_Update);

  /* Slaves Configuration: PWM1 Mode */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 1;

  TIM_OC1Init(TIM3, &TIM_OCInitStructure);

  TIM_OC1Init(TIM4, &TIM_OCInitStructure);

  /* Slave Mode selection: TIM3 */
  TIM_SelectSlaveMode(TIM3, TIM_SlaveMode_Gated);
  TIM_SelectInputTrigger(TIM3, TIM_TS_ITR1);

  /* Select the Master Slave Mode */
  TIM_SelectMasterSlaveMode(TIM3, TIM_MasterSlaveMode_Enable);

  /* Master Mode selection: TIM3 */
  TIM_SelectOutputTrigger(TIM3, TIM_TRGOSource_Update);

  /* Slave Mode selection: TIM4 */
  TIM_SelectSlaveMode(TIM4, TIM_SlaveMode_Gated);
  TIM_SelectInputTrigger(TIM4, TIM_TS_ITR2);

  /* TIM enable counter */
  TIM_Cmd(TIM3, ENABLE);
  TIM_Cmd(TIM2, ENABLE);
  TIM_Cmd(TIM4, ENABLE);

  while (1)
  {
  }
}

/**
  * @brief  Configures the different system clocks.
  * @param  None
  * @retval : None
  */
void RCC_Configuration(void)
{
    /* Setup the microcontroller system. Initialize the Embedded Flash Interface,  
     initialize the PLL and update the SystemFrequency variable. */
  SystemInit();
  
  /* TIM2, TIM3 and TIM4 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 |
                         RCC_APB1Periph_TIM4, ENABLE);


  /* GPIOA and GPIOB clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);
}

/**
  * @brief  Configure the GPIOD Pins.
  * @param  None
  * @retval : None
  */
void GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* GPIOA Configuration: PA0(TIM2 CH1) and PA6(TIM3 CH1) as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* GPIOB Configuration: PB6(TIM4 CH1) as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;

  GPIO_Init(GPIOB, &GPIO_InitStructure);
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param file: pointer to the source file name
  * @param line: assert_param error line source number
  * @retval : None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  while (1)
  {}
}
#endif

/**
  * @}
  */ 

/**
  * @}
  */ 

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
