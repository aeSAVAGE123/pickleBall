//
// Created by Giselle on 2024/3/17.
//

#ifndef CPROJECT_BSP_USART_BLT_H
#define CPROJECT_BSP_USART_BLT_H


#include "stm32f1xx.h"
#include <stdio.h>

#define             BLT_USART_BAUD_RATE                       38400


#define BLT_USARTx 						USART2
#define BLT_USART_CLK				__HAL_RCC_USART2_CLK_ENABLE()

#define BLT_USART_RX_PORT                         GPIOA
#define USART_RX_BLT_CLK_ENABLE      __HAL_RCC_GPIOA_CLK_ENABLE()
#define BLT_USART_RX_PIN                    GPIO_PIN_3
//#define USART_RX_AF                     GPIO_AF7_USART2

#define BLT_USART_TX_PORT                         GPIOA
#define USART_TX_BLT_CLK_ENABLE     __HAL_RCC_GPIOA_CLK_ENABLE()
#define BLT_USART_TX_PIN                    GPIO_PIN_2
//#define USART_TX_AF                   GPIO_AF7_USART2

#define BLT_UARTx_IRQHandler            USART2_IRQHandler
#define BLT_UARTx_IRQ                   USART2_IRQn


#define UART_BUFF_SIZE      1024


typedef struct
{
    volatile    uint16_t datanum;
    uint8_t     uart_buff[UART_BUFF_SIZE];
    uint8_t     receive_data_flag;
}ReceiveData;


void BLT_USART_Config(void);
void Usart_SendStr_length( USART_TypeDef * pUSARTx, uint8_t *str,uint32_t strlen );
void BLT_Usart_SendString(uint8_t *str);

void bsp_USART_Process(void);
char *get_rebuff(uint16_t *len);
void clean_rebuff(void);



#endif //CPROJECT_BSP_USART_BLT_H
