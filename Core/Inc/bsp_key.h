#ifndef __KEY_H
#define	__KEY_H

#include "stm32f1xx.h"
#include "main.h"
//引脚定义
/*******************************************************/
#define KEY1_PIN                  GPIO_PIN_2
#define KEY1_GPIO_PORT            GPIOD
#define KEY1_GPIO_CLK_ENABLE()    __HAL_RCC_GPIOD_CLK_ENABLE()

#define KEY2_PIN                  GPIO_PIN_3
#define KEY2_GPIO_PORT            GPIOD
#define KEY2_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOD_CLK_ENABLE()

#define KEY3_PIN                  GPIO_PIN_4
#define KEY3_GPIO_PORT            GPIOD
#define KEY3_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOD_CLK_ENABLE()

#define KEY4_PIN                  GPIO_PIN_5
#define KEY4_GPIO_PORT            GPIOD
#define KEY4_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOD_CLK_ENABLE()

#define KEY5_PIN                  GPIO_PIN_6
#define KEY5_GPIO_PORT            GPIOD
#define KEY5_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOD_CLK_ENABLE()
/*******************************************************/

/** 按键按下标置宏
   * 按键按下为高电平，设置 KEY_ON=1， KEY_OFF=0
   * 若按键按下为低电平，把宏设置成KEY_ON=0 ，KEY_OFF=1 即可
   */
#define KEY_ON	0
#define KEY_OFF	1

void Key_GPIO_Config(void);
uint8_t Key_Scan(GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin);

#endif /* __LED_H */

