/**
 ******************************************************************************
 * @file    bsp_SysTick.c
 * @author  fire
 * @version V1.0
 * @date    2015-xx-xx
 * @brief   SysTick ϵͳ�δ�ʱ��10us�жϺ�����,�ж�ʱ����������ã�
 *          ���õ��� 1us 10us 1ms �жϡ�
 ******************************************************************************
 * @attention
 *
 * ʵ��ƽ̨:Ұ��  STM32 F407 ������
 * ��̳    :http://www.firebbs.cn
 * �Ա�    :https://fire-stm32.taobao.com
 *
 ******************************************************************************
 */

#include "FreeRTOS.h"
#include "task.h"

#include "./systick/bsp_SysTick.h"

static __IO u32 TimingDelay = 0;

/**
 * @brief  ����ϵͳ�δ�ʱ�� SysTick
 * @param  ��
 * @retval ��
 */
void SysTick_Init(void)
{
  /* SystemFrequency / 1000    1ms�ж�һ��
   * SystemFrequency / 100000	 10us�ж�һ��
   * SystemFrequency / 1000000 1us�ж�һ��
   */
  if (SysTick_Config(SystemCoreClock / 1000000)) // ST3.5.0��汾
  {
    /* Capture error */
    while (1)
      ;
  }

  // �رյδ�ʱ��
  SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
}

/**
 * @brief   us��ʱ����,1usΪһ����λ
 * @param
 *		@arg nTime: Delay_us( 1 ) ��ʵ�ֵ���ʱΪ 1 * 1us = 1us
 * @retval  ��
 */
void Delay_us(__IO u32 nus)
{
  u32 ticks;
  u32 told, tnow, reload, tcnt = 0;
  if ((0x0001 & (SysTick->CTRL)) == 0) // ��ʱ��δ����
    vPortSetupTimerInterrupt();        // ��ʼ����ʱ��

  reload = SysTick->LOAD;                    // ��ȡ��װ�ؼĴ���ֵ
  ticks = nus * (SystemCoreClock / 1000000); // ����ʱ��ֵ

  vTaskSuspendAll();   // ��ֹOS���ȣ���ֹ���us��ʱ
  told = SysTick->VAL; // ��ȡ��ǰ��ֵ�Ĵ���ֵ����ʼʱ��ֵ��
  while (1)
  {
    tnow = SysTick->VAL; // ��ȡ��ǰ��ֵ�Ĵ���ֵ
    if (tnow != told)    // ��ǰֵ�����ڿ�ʼֵ˵�����ڼ���
    {
      if (tnow < told)       // ��ǰֵС�ڿ�ʼ��ֵ��˵��δ�Ƶ�0
        tcnt += told - tnow; // ����ֵ=��ʼֵ-��ǰֵ

      else                            // ��ǰֵ���ڿ�ʼ��ֵ��˵���ѼƵ�0�����¼���
        tcnt += reload - tnow + told; // ����ֵ=��װ��ֵ-��ǰֵ+��ʼֵ  ��
                                      // �Ѵӿ�ʼֵ�Ƶ�0��

      told = tnow; // ���¿�ʼֵ
      if (tcnt >= ticks)
        break; // ʱ�䳬��/����Ҫ�ӳٵ�ʱ��,���˳�.
    }
  }
  xTaskResumeAll(); // �ָ�OS����
}
// SystemCoreClockΪϵͳʱ��(system_stmf4xx.c��)��ͨ��ѡ���ʱ����Ϊ
// systick��ʱ��ʱ�ӣ����ݾ����������

// void Delay_us(__IO u32 nTime)
// {
//   TimingDelay = nTime;

//   // ʹ�ܵδ�ʱ��
//   SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;

//   while (TimingDelay != 0)
//     ;

//   // �رյδ�ʱ��
//   SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
// }

/**
 * @brief  ��ȡ���ĳ���
 * @param  ��
 * @retval ��
 * @attention  �� SysTick �жϺ��� SysTick_Handler()����
 */
void TimingDelay_Decrement(void)
{
  if (TimingDelay != 0x00)
  {
    TimingDelay--;
  }
}
/*********************************************END OF FILE**********************/
