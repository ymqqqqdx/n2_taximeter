/**
  ******************************************************************************
  * @file    Project/Template/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.0.0
  * @date    04/06/2009
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
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
#include "stm32f10x_it.h"
#include "stm32f10x_rtc.h"
#include "stm32f10x_tim.h"
#include <stdio.h>
/** @addtogroup Template_Project
  * @{
  */
extern void RTC_ClearITPendingBit(uint16_t RTC_IT);
extern volatile unsigned int SecStatus;
extern int OneSec;
extern int NoOneSec;


extern float distance;
extern long wait_time;
float speed = 90;
extern int money;

extern u8 key1;
extern u8 key2;

extern u8 kk;
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval : None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval : None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval : None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval : None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval : None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval : None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval : None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval : None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval : None
  */
void SysTick_Handler(void)
{
}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval : None
  */
void RTC_IRQHandler(void)
{
  if(RTC_GetITStatus(RTC_IT_SEC) != RESET)
  {
    RTC_ClearITPendingBit(RTC_IT_SEC);			  //���ȱ������жϣ�����һֱ��Ӧ�ж�
/*    
	switch (SecStatus) {
		case: 0
		GPIO_ResetBits(GPIOE, GPIO_Pin_1);
		break;

		case: 1
		GPIO_SetBits(GPIOE, GPIO_Pin_1);
		break;
		
		default
		break;
	}
*/
	SecStatus = OneSec;							  //���ñ�־λ����ʾ���ڴﵽһ��
    /* Wait until last write operation on RTC registers has finished */
    RTC_WaitForLastTask();
    /* Reset RTC Counter when Time is 23:59:59 */
    if(RTC_GetCounter() == 0x00015180)			  //15180��Ϊ24Сʱ�������
    {
      RTC_SetCounter(0x0);
      /* Wait until last write operation on RTC registers has finished */
      RTC_WaitForLastTask();
    }
  }
} 

void TIM2_IRQHandler(void)
{
	
	if ( TIM_GetITStatus(TIM2 , TIM_IT_Update) != RESET ) {
		TIM_ClearITPendingBit(TIM2 , TIM_FLAG_Update);
		 kk++;
	
	if(key2 ==0)
			wait_time++;
	else
			distance+=speed/3600.0;

			
		//	printf("%s","123321");
		

	}	
}

/**
  * @}

  */ 
  /**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval : None
  */
void EXTI2_IRQHandler(void)
{
	if ( EXTI_GetITStatus(EXTI_Line2) != RESET ) {
		EXTI_ClearITPendingBit(EXTI_Line2);
		/*====LED34-ON=======*/
		key1 = (key1+1)%3;
		Delay(0xfffff);
		Delay(0xfffff);
		Delay(0xfffff);

	}	
}


void EXTI3_IRQHandler(void)
{
	if ( EXTI_GetITStatus(EXTI_Line3) != RESET ) {
		EXTI_ClearITPendingBit(EXTI_Line3);
		/*====LED34-ON=======*/
		key2 = (key2+1)%4;
		//if(key2 == 6)
		//	key2 = 0;
		//speed = speed + 30.0;
		//if(speed == 180.0)
		//	speed =0.0;
		Delay(0xfffff);
		Delay(0xfffff);
		Delay(0xfffff);
	
	}	
}

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
