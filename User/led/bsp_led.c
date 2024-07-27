/**
 ******************************************************************************
 * @file    bsp_led.c
 * @author  fire
 * @version V1.0
 * @date    2015-xx-xx
 * @brief   led
 ******************************************************************************
 * @attention
 *
 * ???é???¨:?°??  STM32 F407 ??·?°?
 * ????    :http://www.firebbs.cn
 * ??±?    :https://fire-stm32.taobao.com
 *
 ******************************************************************************
 */

#include "./led/bsp_led.h"

/**
 * @brief  初始化控制LED的IO
 * @param  无
 * @retval 无
 */
void LED_GPIO_Config(void)
{
    /*定义一个GPIO*/
    GPIO_InitTypeDef GPIO_InitStructure;

    /*时钟*/
    RCC_AHB1PeriphClockCmd(LED1_GPIO_CLK |
                               LED2_GPIO_CLK |
                               LED3_GPIO_CLK,
                           ENABLE);

    /*引脚*/
    GPIO_InitStructure.GPIO_Pin = LED1_PIN;

    /*输出模式*/
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;

    /*推免输出*/
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;

    /*上拉*/
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;

    /*引脚速率为2MHZ */
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;

    /*init*/
    GPIO_Init(LED1_GPIO_PORT, &GPIO_InitStructure);

    // GPIO_InitStructure.GPIO_Pin = LED2_PIN;
    // GPIO_Init(LED2_GPIO_PORT, &GPIO_InitStructure);

    // GPIO_InitStructure.GPIO_Pin = LED3_PIN;
    // GPIO_Init(LED3_GPIO_PORT, &GPIO_InitStructure);

    /*关闭RGB灯*/
    LED1_OFF;
    // LED_RGBOFF;
}
/*********************************************END OF FILE**********************/
