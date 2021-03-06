/**
  ******************************************************************************
  * @file NVIC/IRQ_Channels/main.c 
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
#include "platform_config.h"

/** @addtogroup StdPeriph_Examples
  * @{
  */

/** @addtogroup NVIC_IRQ_Channels
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
NVIC_InitTypeDef NVIC_InitStructure;

/* Private function prototypes -----------------------------------------------*/
void RCC_Configuration(void);
void GPIO_Configuration(void);
void TIM_Configuration(void);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program.
  * @param  None
  * @retval : None
  */
int main(void)
{
  /* Configure the system clocks */
  RCC_Configuration();

  /* Configure GPIOs */
  GPIO_Configuration();

  /* Configure TIMs */
  TIM_Configuration();
       
  /* Configure one bit for preemption priority */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  
  /* Enable the TIM2 Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  /* Enable the TIM3 Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_Init(&NVIC_InitStructure);
 
  /* Enable the TIM4 Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_Init(&NVIC_InitStructure); 
    
  while (1)
  {     
  }
}

/**
  * @brief  Configures the different system clocks
  * @param  None
  * @retval : None
  */
void RCC_Configuration(void)
{
  /* Setup the microcontroller system. Initialize the Embedded Flash Interface,  
     initialize the PLL and update the SystemFrequency variable. */
  SystemInit();

  /* Enable GPIO_LED clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIO_LED, ENABLE);

  /* Enable TIM2, TIM3 and TIM4 clocks */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 |
                         RCC_APB1Periph_TIM4, ENABLE);
}

/**
  * @brief  Configures the used GPIO pins.
  * @param  None
  * @retval : None
  */
void GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Configure GPIO_LED pin 6, GPIO_LED pin 7, GPIO_LED pin 8 as output push-pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIO_LED, &GPIO_InitStructure);
}

/**
  * @brief  Configures the used Timers.
  * @param  None
  * @retval : None
  */
void TIM_Configuration(void)
{ 
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;

  /* TIM2 configuration */
  TIM_TimeBaseStructure.TIM_Period = 0x4AF;          
  TIM_TimeBaseStructure.TIM_Prescaler = 0xEA5F;       
  TIM_TimeBaseStructure.TIM_ClockDivision = 0x0;    
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0x0000;
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
  
  TIM_OCStructInit(&TIM_OCInitStructure);
  /* Output Compare Timing Mode configuration: Channel1 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;
  TIM_OCInitStructure.TIM_Pulse = 0x0;  
  TIM_OC1Init(TIM2, &TIM_OCInitStructure);
  
  /* TIM3 configuration */
  TIM_TimeBaseStructure.TIM_Period = 0x95F;
          
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
  /* Output Compare Timing Mode configuration: Channel1 */
  TIM_OC1Init(TIM3, &TIM_OCInitStructure);
  
  /* TIM4 configuration */
  TIM_TimeBaseStructure.TIM_Period = 0xE0F;  
        
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
  /* Output Compare Timing Mode configuration: Channel1 */
  TIM_OC1Init(TIM4, &TIM_OCInitStructure);

  /* TIM2 enable counter */
  TIM_Cmd(TIM2, ENABLE);
  /* TIM3 enable counter */
  TIM_Cmd(TIM3, ENABLE);
  /* TIM4 enable counter */
  TIM_Cmd(TIM4, ENABLE);

  /* Immediate load of TIM2 Precaler value */
  TIM_PrescalerConfig(TIM2, 0xEA5F, TIM_PSCReloadMode_Immediate);
  /* Immediate load of TIM3 Precaler value */  
  TIM_PrescalerConfig(TIM3, 0xEA5F, TIM_PSCReloadMode_Immediate);
  /* Immediate load of TIM4 Precaler value */
  TIM_PrescalerConfig(TIM4, 0xEA5F, TIM_PSCReloadMode_Immediate);

  /* Clear TIM2 update pending flag */
  TIM_ClearFlag(TIM2, TIM_FLAG_Update);
  /* Clear TIM3 update pending flag */
  TIM_ClearFlag(TIM3, TIM_FLAG_Update);
  /* Clear TIM4 update pending flag */
  TIM_ClearFlag(TIM4, TIM_FLAG_Update);

  /* Enable TIM2 Update interrupt */
  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
  /* Enable TIM3 Update interrupt */
  TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
  /* Enable TIM4 Update interrupt */
  TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
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

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */

/**
  * @}
  */

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
