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
 * ???��???��:?��??  STM32 F407 ??��?��?
 * ????    :http://www.firebbs.cn
 * ??��?    :https://fire-stm32.taobao.com
 *
 ******************************************************************************
 */

#include "./led/bsp_led.h"

/**
 * @brief  ��ʼ������LED��IO
 * @param  ��
 * @retval ��
 */
void LED_GPIO_Config(void)
{
    /*����һ��GPIO*/
    GPIO_InitTypeDef GPIO_InitStructure;

    /*ʱ��*/
    RCC_AHB1PeriphClockCmd(LED1_GPIO_CLK |
                               LED2_GPIO_CLK |
                               LED3_GPIO_CLK,
                           ENABLE);

    /*����*/
    GPIO_InitStructure.GPIO_Pin = LED1_PIN;

    /*���ģʽ*/
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;

    /*�������*/
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;

    /*����*/
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;

    /*��������Ϊ2MHZ */
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;

    /*init*/
    GPIO_Init(LED1_GPIO_PORT, &GPIO_InitStructure);

    // GPIO_InitStructure.GPIO_Pin = LED2_PIN;
    // GPIO_Init(LED2_GPIO_PORT, &GPIO_InitStructure);

    // GPIO_InitStructure.GPIO_Pin = LED3_PIN;
    // GPIO_Init(LED3_GPIO_PORT, &GPIO_InitStructure);

    /*�ر�RGB��*/
    LED1_OFF;
    // LED_RGBOFF;
}
/*********************************************END OF FILE**********************/
