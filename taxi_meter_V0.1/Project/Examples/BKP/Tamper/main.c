/**
  ******************************************************************************
  * @file BKP/Tamper/main.c 
  * @author  MCD Application Team
  * @version  V3.0.0
  * @date  04/06/2009
  * @brief  Main program body.
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
#include "main.h"

/** @addtogroup StdPeriph_Examples
  * @{
  */

/** @addtogroup BKP_Tamper
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
#ifdef USE_STM3210B_EVAL
uint16_t BKPDataReg[BKP_DR_NUMBER] =
  {
    BKP_DR1, BKP_DR2, BKP_DR3, BKP_DR4, BKP_DR5, BKP_DR6, BKP_DR7, BKP_DR8,
    BKP_DR9, BKP_DR10
  };
#elif defined USE_STM3210E_EVAL
uint16_t BKPDataReg[BKP_DR_NUMBER] =
  {
    BKP_DR1, BKP_DR2, BKP_DR3, BKP_DR4, BKP_DR5, BKP_DR6, BKP_DR7, BKP_DR8,
    BKP_DR9, BKP_DR10, BKP_DR11, BKP_DR12, BKP_DR13, BKP_DR14, BKP_DR15, BKP_DR16,
    BKP_DR17, BKP_DR18, BKP_DR19, BKP_DR20, BKP_DR21, BKP_DR22, BKP_DR23, BKP_DR24,
    BKP_DR25, BKP_DR26, BKP_DR27, BKP_DR28, BKP_DR29, BKP_DR30, BKP_DR31, BKP_DR32,
    BKP_DR33, BKP_DR34, BKP_DR35, BKP_DR36, BKP_DR37, BKP_DR38, BKP_DR39, BKP_DR40,
    BKP_DR41, BKP_DR42
  };  
#endif /* USE_STM3210B_EVAL */

/* Private function prototypes -----------------------------------------------*/
void RCC_Configuration(void);
void GPIO_Configuration(void);
void NVIC_Configuration(void);
void WriteToBackupReg(uint16_t FirstBackupData);
uint32_t CheckBackupReg(uint16_t FirstBackupData);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program.
  * @param  None
  * @retval : None
  */
int main(void)
{
  /* System Clocks Configuration */
  RCC_Configuration();
  
  /* NVIC configuration */
  NVIC_Configuration();
    
  /* GPIO configuration */
  GPIO_Configuration();

  /* Enable PWR and BKP clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);

  /* Enable write access to Backup domain */
  PWR_BackupAccessCmd(ENABLE);

  /* Clear Tamper pin Event(TE) pending flag */
  BKP_ClearFlag();

  /* Tamper pin active on low level */
  BKP_TamperPinLevelConfig(BKP_TamperPinLevel_Low);

  /* Enable Tamper interrupt */
  BKP_ITConfig(ENABLE);

  /* Enable Tamper pin */
  BKP_TamperPinCmd(ENABLE);
   
  /* Write data to Backup DRx registers */
  WriteToBackupReg(0xA53C);
  /* Check if the written data are correct */
  if(CheckBackupReg(0xA53C) == 0x00)
  {
    /* Turn on led connected to GPIO_LED Pin6 */
    GPIO_Write(GPIO_LED, GPIO_Pin_6);
  }
  else
  {
    /* Turn on led connected to GPIO_LED Pin7 */
    GPIO_Write(GPIO_LED, GPIO_Pin_7);
  }
        
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

  /* Enable GPIO_LED clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIO_LED, ENABLE);
}

/**
  * @brief  Configures the different GPIO ports.
  * @param  None
  * @retval : None
  */
void GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Configure GPIO_LED Pin 6, Pin 7, Pin 8 and Pin 9 as Output push-pull ----*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIO_LED, &GPIO_InitStructure);
}

/**
  * @brief  Configures NVIC and Vector Table base location.
  * @param  None
  * @retval : None
  */
void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

  /* Enable TAMPER IRQChannel */
  NVIC_InitStructure.NVIC_IRQChannel = TAMPER_IRQn;
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
  * @brief  Writes data Backup DRx registers.
  * @param FirstBackupData: data to be written to Backup data registers.
  * @retval : None
  */
void WriteToBackupReg(uint16_t FirstBackupData)
{
  uint32_t index = 0;

  for (index = 0; index < BKP_DR_NUMBER; index++)
  {
    BKP_WriteBackupRegister(BKPDataReg[index], FirstBackupData + (index * 0x5A));
  }  
}

/**
  * @brief  Checks if the Backup DRx registers values are correct or not.
  * @param FirstBackupData: data to be compared with Backup data registers.
  * @retval : 
  *         - 0: All Backup DRx registers values are correct
  *         - Value different from 0: Number of the first Backup register
  *           which value is not correct
  */
uint32_t CheckBackupReg(uint16_t FirstBackupData)
{
  uint32_t index = 0;

  for (index = 0; index < BKP_DR_NUMBER; index++)
  {
    if (BKP_ReadBackupRegister(BKPDataReg[index]) != (FirstBackupData + (index * 0x5A)))
    {
      return (index + 1);
    }
  }

  return 0;  
}

/**
  * @brief  Checks if the Backup DRx registers are reset or not.
  * @param  None
  * @retval : 
  *          - 0: All Backup DRx registers are reset
  *          - Value different from 0: Number of the first Backup register
  *            not reset
  */
uint32_t IsBackupRegReset(void)
{
  uint32_t index = 0;

  for (index = 0; index < BKP_DR_NUMBER; index++)
  {
    if (BKP_ReadBackupRegister(BKPDataReg[index]) != 0x0000)
    {
      return (index + 1);
    }
  }

  return 0;  
}

/**
  * @}
  */ 

/**
  * @}
  */ 

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
