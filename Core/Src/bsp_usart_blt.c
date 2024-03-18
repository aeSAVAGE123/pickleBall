//
// Created by Giselle on 2024/3/17.
//
/**
  ******************************************************************************
  * @file    bsp_usart1.c
  * @author  fire
  * @version V1.0
  * @date    2013-xx-xx
  * @brief   HC05串口驱动
  ******************************************************************************
  * @attention
  *
  * 实验平台:野火 霸道 STM32 开发板
  * 论坛    :http://www.firebbs.cn
  * 淘宝    :https://fire-stm32.taobao.com
  *
  ******************************************************************************
  */

#include "bsp_usart_blt.h"
#include <stdarg.h>

UART_HandleTypeDef USART_InitStructure;
extern ReceiveData BLT_USART_ReceiveData;
#define UART_BUFF_SIZE2      1024
volatile    uint16_t uart_p2 = 0;
uint8_t     uart_buff2[UART_BUFF_SIZE2];

/// 配置USART接收中断
//static void BLT_NVIC_Configuration(void)
//{
//    NVIC_InitTypeDef NVIC_InitStructure;
//    /* Configure the NVIC Preemption Priority Bits */
//    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
//
//    /* Enable the USARTy Interrupt */
//    NVIC_InitStructure.NVIC_IRQChannel = BLT_USART_IRQ;
//    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =1;
//    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
//    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//    NVIC_Init(&NVIC_InitStructure);
//}

/*
 * 函数名：USARTx_Config
 * 描述  ：USART GPIO 配置,工作模式配置
 * 输入  ：无
 * 输出  : 无
 * 调用  ：外部调用
 */
void BLT_USART_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
//    USART_InitTypeDef USART_InitStructure;

    /* config USART2 clock */
    BLT_USART_CLK;
    USART_RX_BLT_CLK_ENABLE;
    USART_TX_BLT_CLK_ENABLE;

    /* USART2 GPIO config */
    /* Configure USART2 Tx (PA.02) as alternate function push-pull */
    GPIO_InitStructure.Pin = BLT_USART_TX_PIN;
    GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(BLT_USART_TX_PORT, &GPIO_InitStructure);

    /* Configure USART2 Rx (PA.03) as input floating */
    GPIO_InitStructure.Pin = BLT_USART_RX_PIN;
    GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
    HAL_GPIO_Init(BLT_USART_RX_PORT, &GPIO_InitStructure);

    /* USART2 mode config */
    USART_InitStructure.Instance = BLT_USARTx;
    USART_InitStructure.Init.BaudRate = BLT_USART_BAUD_RATE;
    USART_InitStructure.Init.WordLength = USART_WORDLENGTH_8B;
    USART_InitStructure.Init.StopBits = USART_STOPBITS_1;
    USART_InitStructure.Init.Parity = USART_PARITY_NONE ;
    USART_InitStructure.Init.Mode = UART_MODE_TX_RX;
    USART_InitStructure.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    HAL_UART_Init(&USART_InitStructure);

    /*	配置中断优先级 */
//    BLT_NVIC_Configuration();
    /* 使能串口2接收中断 */
    /*串口6中断初始化 */
    HAL_NVIC_SetPriority(BLT_UARTx_IRQ, 0, 0);
    HAL_NVIC_EnableIRQ(BLT_UARTx_IRQ);
    /*配置串口接收中断 */
    __HAL_UART_ENABLE_IT(&USART_InitStructure,USART_IT_RXNE);
}

/*****************  发送字符串 **********************/
void BLT_Usart_SendString(uint8_t *str)
{
    unsigned int k=0;
    do
    {
        HAL_UART_Transmit( &USART_InitStructure,(uint8_t *)(str + k) ,1,1000);
        k++;
    } while(*(str + k)!='\0');

}


void bsp_USART_Process(void)
{
    if(uart_p2<UART_BUFF_SIZE2)
    {
        if(__HAL_UART_GET_IT_SOURCE(&USART_InitStructure,UART_IT_RXNE) != RESET)
        {
            HAL_UART_Receive(&USART_InitStructure, (uint8_t *)&uart_buff2[uart_p2], 1, 1000);
            uart_p2++;
        }
    }
    else
    {
        clean_rebuff();
    }
    HAL_UART_IRQHandler(&USART_InitStructure);
}



//获取接收到的数据和长度
char *get_rebuff(uint16_t *len)
{
    *len = uart_p2;
    return (char *)&uart_buff2;
}

//清空缓冲区
void clean_rebuff(void)
{

    uint16_t i=UART_BUFF_SIZE2+1;
    uart_p2 = 0;
    while(i)
        uart_buff2[--i]=0;

}













/******************* (C) COPYRIGHT 2012 WildFire Team *****END OF FILE************/
