/**
 ******************************************************************************
 * @file    Project/STM32F4xx_StdPeriph_Templates/stm32f4xx_it.c
 * @author  MCD Application Team
 * @version V1.5.0
 * @date    06-March-2015
 * @brief   Main Interrupt Service Routines.
 *          This file provides template for all exceptions handler and
 *          peripherals interrupt service routine.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT 2015 STMicroelectronics</center></h2>
 *
 * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
 * You may not use this file except in compliance with the License.
 * You may obtain a copy of the License at:
 *
 *        http://www.st.com/software_license_agreement_liberty_v2
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_it.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "task.h"

#include "./systick/bsp_SysTick.h"
#include "./led/bsp_breath_led.h"
#include "./usart/bsp_debug_usart.h"
#include "./DHT11/bsp_dht11.h"

#define TASK_DELAY_NUM 2 // ���������
#define TASK_DELAY_0 200 // ����0��ʱ200*10�����ִ�У���ȡDHT����������
#define TASK_DELAY_1 50  // ����1��ʱ50*10�����ִ��

uint32_t Task_Delay_Group[TASK_DELAY_NUM]; // �������飬������ʱ�����ж��Ƿ�ִ�ж�Ӧ����
/* ��������������ɱ�־ */
// - ��־��1��ʾ��ɶ�ȡ,����ѭ����������
// - ��־��0��ʾδ��ɶ�ȡ
// - ��־-1��ʾ��ȡ����
int read_dht11_finish;

// �ⲿ����
extern DHT11_Data_TypeDef DHT11_Data;

/** @addtogroup STM32F429I_DISCOVERY_Examples
 * @{
 */

/** @addtogroup FMC_SDRAM
 * @{
 */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
 * @brief  This function handles NMI exception.
 * @param  None
 * @retval None
 */
void NMI_Handler(void)
{
}

/**
 * @brief  This function handles Hard Fault exception.
 * @param  None
 * @retval None
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
 * @retval None
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
 * @retval None
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
 * @retval None
 */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
 * @brief  This function handles Debug Monitor exception.
 * @param  None
 * @retval None
 */
void DebugMon_Handler(void)
{
}

// /**
//   * @brief  This function handles SVCall exception.
//   * @param  None
//   * @retval None
//   */
// void SVC_Handler(void)
// {}

// /**
//   * @brief  This function handles PendSV_Handler exception.
//   * @param  None
//   * @retval None
//   */
// void PendSV_Handler(void)
// {}

/**
 * @brief  This function handles SysTick Handler.
 * @param  None
 * @retval None
 */
extern void xPortSysTickHandler(void);
// systick�жϷ�����
void SysTick_Handler(void)
{
#if (INCLUDE_xTaskGetSchedulerState == 1)
  if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
  {
#endif /* INCLUDE_xTaskGetSchedulerState */
    xPortSysTickHandler();
#if (INCLUDE_xTaskGetSchedulerState == 1)
  }
#endif /* INCLUDE_xTaskGetSchedulerState */
}
/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f429_439xx.s).                         */
/******************************************************************************/
// void DEBUG_USART_IRQHandler(void)
//{
//   uint8_t ucTemp;
//	if(USART_GetITStatus(DEBUG_USART,USART_IT_RXNE)!=RESET)
//	{
//		ucTemp = USART_ReceiveData( DEBUG_USART );
//     USART_SendData(DEBUG_USART,ucTemp);
//	}
// }
//
/**
 * @}
 */

/**
 * @}
 */

extern uint16_t indexWave[];

void BRE_TIM_IRQHandler(void)
{
  static uint16_t pwm_index = 0;  // PWM���
  static uint16_t period_cnt = 0; // ���ڼ���������

  if (TIM_GetITStatus(BRE_TIM, TIM_IT_Update) != RESET) // TIM_IT_Update
  {
    period_cnt++;

    BRE_TIM->BRE_LED_CCRx = indexWave[pwm_index]; // ����PWM���޸Ķ�ʱ���ıȽϼĴ���ֵ

    // ÿ��PWM���е�ÿ��Ԫ��ʹ��period_class��
    if (period_cnt > period_class)
    {

      pwm_index++; // ��־PWM��ָ����һ��Ԫ��

      // ��PWM���е�ÿ��Ԫ��ʹ��period_class��
      if (pwm_index >= POINT_NUM)
      {
        pwm_index = 0;
      }

      period_cnt = 0; // ��ֵ���ڼ�����־
    }
    else
    {
    }

    TIM_ClearITPendingBit(BRE_TIM, TIM_IT_Update); // ����Ҫ����жϱ�־λ
  }
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
