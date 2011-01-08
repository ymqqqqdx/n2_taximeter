/**
  ******************************************************************************
  * @file    Project/Template/main.c 
  * @author  MCD Application Team
  * @version V3.0.0
  * @date    04/06/2009
  * @brief   Main program body
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
#include "stm32f10x_tim.h"
#include "misc.h"
#include "lcd.h"

 float distance;
 long wait_time;
 float speed;
float money;

 u8 kk;

 void FSMC_LCD_Init(void);

ITStatus EXTIStatus;
TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
															
void GPIO_Config(void);
void Timer_Config(void);
void NVIC_Config(void);
void EXTI_Config(void);
/**
  * @brief  Delay program.
  * @param  None
  * @retval : None
  */
void  Delay (u32 nCount)
{
  for(; nCount != 0; nCount--);
}

/**
  * @brief  Main program.
  * @param  None
  * @retval : None
  */

u8 key1;

int main(void)
{

	/* Setup STM32 system (clock, PLL and Flash configuration) */
	/* Enable the FSMC Clock */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_FSMC, ENABLE);    

	/* Configure FSMC */
	FSMC_LCD_Init();

	/* Init for LCD */
	LCD_Setup();


	SystemInit();
	GPIO_Config();
	NVIC_Config();
	EXTI_Config();
	Timer_Config();

	/* Infinite loop */
	distance = 0;
	wait_time = 50;
	speed = 100;
	while (1) {
		
				if(key1)
					display();
				else
				{
				GPIO_SetBits(GPIOB , GPIO_Pin_8);
				GPIO_SetBits(GPIOB , GPIO_Pin_9);
				GPIO_SetBits(GPIOE , GPIO_Pin_0);
				GPIO_ResetBits(GPIOE , GPIO_Pin_1);
				Delay(0xfffff);
				Delay(0xfffff);
				Delay(0xfffff);
				GPIO_ResetBits(GPIOB , GPIO_Pin_8);
				GPIO_ResetBits(GPIOB , GPIO_Pin_9);
				GPIO_ResetBits(GPIOE , GPIO_Pin_0);
				GPIO_SetBits(GPIOE , GPIO_Pin_1);
				Delay(0xfffff);
				Delay(0xfffff);
				Delay(0xfffff);
				}

	}
}

/**
  * @brief  EXTI_Config Program.
  * @param  None
  * @retval : None
  */
void EXTI_Config(void)
{
	EXTI_InitTypeDef EXTI_InitStructure;
	//EXTI_StructInit(&EXTI_InitStructure);
				 	
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOE, GPIO_PinSource2);	//管脚选择
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOE, GPIO_PinSource3);
	EXTI_ClearITPendingBit(EXTI_Line3);
	EXTI_ClearITPendingBit(EXTI_Line2);

	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_Line = EXTI_Line2 | EXTI_Line3;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
	//EXTI_Cmd(EXTI , ENABLE);
}

/**
  * @brief  Configures the FSMC and GPIOs to interface with the SRAM memory.
  *         This function must be called before any write/read operation
  *         on the SRAM.
  * @param  None 
  * @retval : None
  */
void FSMC_LCD_Init(void)
{
  FSMC_NORSRAMInitTypeDef  FSMC_NORSRAMInitStructure;
  FSMC_NORSRAMTimingInitTypeDef  FSMC_TimingInitStructure;

  FSMC_TimingInitStructure.FSMC_AddressSetupTime = 0x02;
  FSMC_TimingInitStructure.FSMC_AddressHoldTime = 0x00;
  FSMC_TimingInitStructure.FSMC_DataSetupTime = 0x05;
  FSMC_TimingInitStructure.FSMC_BusTurnAroundDuration = 0x00;
  FSMC_TimingInitStructure.FSMC_CLKDivision = 0x00;
  FSMC_TimingInitStructure.FSMC_DataLatency = 0x00;
  FSMC_TimingInitStructure.FSMC_AccessMode = FSMC_AccessMode_B;

  FSMC_NORSRAMInitStructure.FSMC_Bank = FSMC_Bank1_NORSRAM1;
  FSMC_NORSRAMInitStructure.FSMC_DataAddressMux = FSMC_DataAddressMux_Disable;
  FSMC_NORSRAMInitStructure.FSMC_MemoryType = FSMC_MemoryType_NOR;
  FSMC_NORSRAMInitStructure.FSMC_MemoryDataWidth = FSMC_MemoryDataWidth_16b;
  FSMC_NORSRAMInitStructure.FSMC_BurstAccessMode = FSMC_BurstAccessMode_Disable;
  FSMC_NORSRAMInitStructure.FSMC_WaitSignalPolarity = FSMC_WaitSignalPolarity_Low;
  FSMC_NORSRAMInitStructure.FSMC_WrapMode = FSMC_WrapMode_Disable;
  FSMC_NORSRAMInitStructure.FSMC_WaitSignalActive = FSMC_WaitSignalActive_BeforeWaitState;
  FSMC_NORSRAMInitStructure.FSMC_WriteOperation = FSMC_WriteOperation_Enable;
  FSMC_NORSRAMInitStructure.FSMC_WaitSignal = FSMC_WaitSignal_Disable;
  FSMC_NORSRAMInitStructure.FSMC_ExtendedMode = FSMC_ExtendedMode_Disable;
  FSMC_NORSRAMInitStructure.FSMC_WriteBurst = FSMC_WriteBurst_Disable;
  FSMC_NORSRAMInitStructure.FSMC_ReadWriteTimingStruct = &FSMC_TimingInitStructure;
  FSMC_NORSRAMInitStructure.FSMC_WriteTimingStruct = &FSMC_TimingInitStructure;	  

  FSMC_NORSRAMInit(&FSMC_NORSRAMInitStructure); 
  FSMC_NORSRAMCmd(FSMC_Bank1_NORSRAM1, ENABLE);  
}


/**
  * @brief  GPIO_Config program.
  * @param  None
  * @retval : None
  */
void GPIO_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC |
                         RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOE | RCC_APB2Periph_AFIO, ENABLE); 
						 
/**
 *	LED1 -> PB8	 ,	LED2 -> PB9 , LED3 -> PE0 , LED4 -> PE1
 *  蜂鸣器 -> PC5
 */	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_8 |GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
  GPIO_Init(GPIOB, &GPIO_InitStructure);					 

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 |GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
  GPIO_Init(GPIOE, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
  GPIO_Init(GPIOC, &GPIO_InitStructure);

/**
 *	LED1-4-OFF
 */
  GPIO_SetBits(GPIOB , GPIO_Pin_8);
  GPIO_SetBits(GPIOB , GPIO_Pin_9);
  GPIO_SetBits(GPIOE , GPIO_Pin_0);
  GPIO_SetBits(GPIOE , GPIO_Pin_1);

}


/**
  * @brief  EXTI_Config Program.
  * @param  None
  * @retval : None
  */
void Timer_Config(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 , ENABLE);

	TIM_DeInit(TIM2);

	TIM_TimeBaseStructure.TIM_Period=2000;		 					//自动重装载寄存器周期的值(计数值)
																	//累计 TIM_Period个频率后产生一个更新或者中断
	TIM_TimeBaseStructure.TIM_Prescaler= (36000 - 1);				//时钟预分频数   例如：时钟频率=72MHZ/(时钟预分频+1)
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 			//采样分频
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; 		//向上计数模式
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	TIM_ClearFlag(TIM2, TIM_FLAG_Update);							//清除溢出中断标志
	//TIM_PrescalerConfig(TIM2,0x8C9F,TIM_PSCReloadMode_Immediate);	//时钟分频系数36000，所以定时器时钟为2K
	//TIM_ARRPreloadConfig(TIM2, DISABLE);							//禁止ARR预装载缓冲器
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);

	TIM_Cmd(TIM2, ENABLE);											//开启时钟

}


/**
  * @brief  Configures the nested vectored interrupt controller.
  * @param  None
  * @retval : None
  */
void NVIC_Config(void)
{
	NVIC_InitTypeDef NVIC_InitStructure; 

	 
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);  													
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQChannel;	  //通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;//
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;	  //
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);  													
	NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQChannel;	  //通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;//
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;	  //
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);  													
	NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQChannel;	  //通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;//
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;	  //
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);  


	  
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);
  NVIC_InitStructure.NVIC_IRQChannel = RTC_IRQChannel;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
 
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


/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
