/**
 ******************************************************************************
 * @file    bsp_SysTick.c
 * @author  fire
 * @version V1.0
 * @date    2015-xx-xx
 * @brief   SysTick 系统滴答时钟10us中断函数库,中断时间可自由配置，
 *          常用的有 1us 10us 1ms 中断。
 ******************************************************************************
 * @attention
 *
 * 实验平台:野火  STM32 F407 开发板
 * 论坛    :http://www.firebbs.cn
 * 淘宝    :https://fire-stm32.taobao.com
 *
 ******************************************************************************
 */

#include "FreeRTOS.h"
#include "task.h"

#include "./systick/bsp_SysTick.h"

static __IO u32 TimingDelay = 0;

/**
 * @brief  启动系统滴答定时器 SysTick
 * @param  无
 * @retval 无
 */
void SysTick_Init(void)
{
  /* SystemFrequency / 1000    1ms中断一次
   * SystemFrequency / 100000	 10us中断一次
   * SystemFrequency / 1000000 1us中断一次
   */
  if (SysTick_Config(SystemCoreClock / 1000000)) // ST3.5.0库版本
  {
    /* Capture error */
    while (1)
      ;
  }

  // 关闭滴答定时器
  SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
}

/**
 * @brief   us延时程序,1us为一个单位
 * @param
 *		@arg nTime: Delay_us( 1 ) 则实现的延时为 1 * 1us = 1us
 * @retval  无
 */
void Delay_us(__IO u32 nus)
{
  u32 ticks;
  u32 told, tnow, reload, tcnt = 0;
  if ((0x0001 & (SysTick->CTRL)) == 0) // 定时器未工作
    vPortSetupTimerInterrupt();        // 初始化定时器

  reload = SysTick->LOAD;                    // 获取重装载寄存器值
  ticks = nus * (SystemCoreClock / 1000000); // 计数时间值

  vTaskSuspendAll();   // 阻止OS调度，防止打断us延时
  told = SysTick->VAL; // 获取当前数值寄存器值（开始时数值）
  while (1)
  {
    tnow = SysTick->VAL; // 获取当前数值寄存器值
    if (tnow != told)    // 当前值不等于开始值说明已在计数
    {
      if (tnow < told)       // 当前值小于开始数值，说明未计到0
        tcnt += told - tnow; // 计数值=开始值-当前值

      else                            // 当前值大于开始数值，说明已计到0并重新计数
        tcnt += reload - tnow + told; // 计数值=重装载值-当前值+开始值  （
                                      // 已从开始值计到0）

      told = tnow; // 更新开始值
      if (tcnt >= ticks)
        break; // 时间超过/等于要延迟的时间,则退出.
    }
  }
  xTaskResumeAll(); // 恢复OS调度
}
// SystemCoreClock为系统时钟(system_stmf4xx.c中)，通常选择该时钟作为
// systick定时器时钟，根据具体情况更改

// void Delay_us(__IO u32 nTime)
// {
//   TimingDelay = nTime;

//   // 使能滴答定时器
//   SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;

//   while (TimingDelay != 0)
//     ;

//   // 关闭滴答定时器
//   SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
// }

/**
 * @brief  获取节拍程序
 * @param  无
 * @retval 无
 * @attention  在 SysTick 中断函数 SysTick_Handler()调用
 */
void TimingDelay_Decrement(void)
{
  if (TimingDelay != 0x00)
  {
    TimingDelay--;
  }
}
/*********************************************END OF FILE**********************/
