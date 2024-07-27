#ifndef __BREATH_LED_H
#define __BREATH_LED_H

#include "stm32f4xx.h"

/*PWM���еĵ���*/
extern uint16_t POINT_NUM;
// ����������ε�Ƶ��
extern __IO uint16_t period_class;

#define RED_LIGHT 1
#define GREEN_LIGHT 2
#define BLUE_LIGHT 3

/*Ҫʹ��ʲô��ɫ�ĺ����ƣ���ѡRED_LIGHT��GREEN_LIGHT��BLUE_LIGHT*/
#define LIGHT_COLOR GREEN_LIGHT

/********************��ʱ��ͨ��**************************/
#if LIGHT_COLOR == RED_LIGHT
// R ��ɫ��
#define BRE_TIM TIM10
#define BRE_TIM_CLK RCC_APB2Periph_TIM10
#define BRE_TIM_APBxClock_FUN RCC_APB2PeriphClockCmd

#define BRE_TIM_IRQn TIM1_UP_TIM10_IRQn
#define BRE_TIM_IRQHandler TIM1_UP_TIM10_IRQHandler

/*����˵����c�ļ�*/
/*����ͨ�ö�ʱ����ʱ��ΪHCLK/4������ΪHCLK/2*/
#define BRE_TIM_PRESCALER (470 - 1)

#define BRE_LED_PIN GPIO_PIN_6
#define BRE_LED_GPIO_PORT GPIOF
#define BRE_LED_GPIO_CLK RCC_AHB1Periph_GPIOF
#define BRE_LED_PINSOURCE GPIO_PinSource6
#define BRE_LED_AF GPIO_AF_TIM10

// ͨ���ȽϼĴ�������TIMx->CCRx��ʽ�ɷ��ʸüĴ����������µıȽ�ֵ������ռ�ձ�
// �Ժ��װ��ʹ��������ʽ��BRE_TIMx->BRE_LED_CCRx ���ɷ��ʸ�ͨ���ıȽϼĴ���
#define BRE_LED_CCRx CCR1
#define BRE_LED_TIM_CHANNEL TIM_Channel_1

#define BRE_TIM_OCxInit TIM_OC1Init // ͨ��ѡ��1~4
#define BRE_TIM_OCxPreloadConfig TIM_OC1PreloadConfig

#elif LIGHT_COLOR == GREEN_LIGHT
// G ��ɫ��
#define BRE_TIM TIM11
#define BRE_TIM_CLK RCC_APB2Periph_TIM11
#define BRE_TIM_APBxClock_FUN RCC_APB2PeriphClockCmd

#define BRE_TIM_IRQn TIM1_TRG_COM_TIM11_IRQn
#define BRE_TIM_IRQHandler TIM1_TRG_COM_TIM11_IRQHandler

/*����˵����c�ļ�*/
/*����ͨ�ö�ʱ����ʱ��ΪHCLK/4������ΪHCLK/2*/
#define BRE_TIM_PRESCALER (470 - 1)

#define BRE_LED_PIN GPIO_Pin_7
#define BRE_LED_GPIO_PORT GPIOF
#define BRE_LED_GPIO_CLK RCC_AHB1Periph_GPIOF
#define BRE_LED_PINSOURCE GPIO_PinSource7
#define BRE_LED_AF GPIO_AF_TIM11

// ͨ���ȽϼĴ�������TIMx->CCRx��ʽ�ɷ��ʸüĴ����������µıȽ�ֵ������ռ�ձ�
// �Ժ��װ��ʹ��������ʽ��BRE_TIM->BRE_LED_CCRx ���ɷ��ʸ�ͨ���ıȽϼĴ���
#define BRE_LED_CCRx CCR1
#define BRE_LED_TIM_CHANNEL TIM_Channel_1

#define BRE_TIM_OCxInit TIM_OC1Init // ͨ��ѡ��1~4
#define BRE_TIM_OCxPreloadConfig TIM_OC1PreloadConfig

#elif LIGHT_COLOR == BLUE_LIGHT
// B ��ɫ��
#define BRE_TIM TIM13
#define BRE_TIM_CLK RCC_APB1Periph_TIM13
#define BRE_TIM_APBxClock_FUN RCC_APB1PeriphClockCmd

#define BRE_TIM_IRQn TIM8_UP_TIM13_IRQn
#define BRE_TIM_IRQHandler TIM8_UP_TIM13_IRQHandler

/*����˵����c�ļ�*/
/*����ͨ�ö�ʱ����ʱ��ΪHCLK/4������ΪHCLK/2*/
#define BRE_TIM_PRESCALER (235 - 1)

#define BRE_LED_PIN GPIO_Pin_8
#define BRE_LED_GPIO_PORT GPIOF
#define BRE_LED_GPIO_CLK RCC_AHB1Periph_GPIOF
#define BRE_LED_PINSOURCE GPIO_PinSource8
#define BRE_LED_AF GPIO_AF_TIM13

// ͨ���ȽϼĴ�������TIMx->CCRx��ʽ�ɷ��ʸüĴ����������µıȽ�ֵ������ռ�ձ�
// �Ժ��װ��ʹ��������ʽ��BRE_TIM->BRE_LED_CCRx ���ɷ��ʸ�ͨ���ıȽϼĴ���
#define BRE_LED_CCRx CCR1
#define BRE_LED_TIM_CHANNEL TIM_Channel_1

#define BRE_TIM_OCxInit TIM_OC1Init // ͨ��ѡ��1~4
#define BRE_TIM_OCxPreloadConfig TIM_OC1PreloadConfig

#endif

void BreathLED_Config(void);

#endif /* __BREATH_LED_H */
