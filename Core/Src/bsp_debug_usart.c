//#include "bsp_debug_usart.h"
//
//UART_HandleTypeDef UartHandle2;
//
////串口接收数组
//unsigned char UART_RxBuffer[UART_RX_BUFFER_SIZE];
////串口接收数组指针
//unsigned char UART_RxPtr;
///* 命令接收完成 */
//uint8_t receive_cmd = 0;
//
///**
//  * @brief  DEBUG_USART GPIO 配置,工作模式配置。115200 8-N-1
//  * @param  无
//  * @retval 无
//  */
//void DEBUG_USART_Config(void)
//{
//
//    UartHandle2.Instance          = DEBUG_USART;
//
//    UartHandle2.Init.BaudRate     = DEBUG_USART_BAUDRATE;
//    UartHandle2.Init.WordLength   = UART_WORDLENGTH_8B;
//    UartHandle2.Init.StopBits     = UART_STOPBITS_1;
//    UartHandle2.Init.Parity       = UART_PARITY_NONE;
//    UartHandle2.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
//    UartHandle2.Init.Mode         = UART_MODE_TX_RX;
//
//    HAL_UART_Init(&UartHandle2);
//
//    /*使能串口接收断 */
//    __HAL_UART_ENABLE_IT(&UartHandle2,UART_IT_RXNE);
//
////    HAL_NVIC_SetPriority(DEBUG_USART_IRQ, 6, 0);	// 抢占优先级0，子优先级1
////    HAL_NVIC_EnableIRQ(DEBUG_USART_IRQ );		      // 使能USART1中断通道
//}
//
//
///**
//  * @brief UART MSP 初始化
//  * @param huart: UART handle
//  * @retval 无
//  */
//void HAL_UART_MspInit(UART_HandleTypeDef *huart)
//{
//    GPIO_InitTypeDef  GPIO_InitStruct;
//
//    DEBUG_USART_CLK_ENABLE();
//
//    DEBUG_USART_RX_GPIO_CLK_ENABLE();
//    DEBUG_USART_TX_GPIO_CLK_ENABLE();
//
///**USART1 GPIO Configuration
//  PA9     ------> USART1_TX
//  PA10    ------> USART1_RX
//  */
//    /* 配置Tx引脚  */
//    GPIO_InitStruct.Pin = DEBUG_USART_TX_PIN;
//    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//    GPIO_InitStruct.Pull = GPIO_PULLUP;
//    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
//    HAL_GPIO_Init(DEBUG_USART_TX_GPIO_PORT, &GPIO_InitStruct);
//
//    /* 配置Rx引脚 */
//    GPIO_InitStruct.Mode = GPIO_MODE_AF_INPUT;
//    GPIO_InitStruct.Pin = DEBUG_USART_RX_PIN;
//    HAL_GPIO_Init(DEBUG_USART_RX_GPIO_PORT, &GPIO_InitStruct);
//
////    HAL_NVIC_SetPriority(DEBUG_USART_IRQ ,5,0);	//抢占优先级0，子优先级1
////    HAL_NVIC_EnableIRQ(DEBUG_USART_IRQ );		    //使能USART1中断通道       //使能USART1中断通道
//}
//
//
//
////清空接收缓冲
//void uart_FlushRxBuffer(void)
//{
//    UART_RxPtr = 0;
//    UART_RxBuffer[UART_RxPtr] = 0;
//}
//
///*****************  发送字符 **********************/
//void Usart_SendByte(uint8_t str)
//{
//    HAL_UART_Transmit(&UartHandle2, &str, 1, 1000);
//}
//
///*****************  发送字符串 **********************/
//void Usart_SendString(uint8_t *str)
//{
//    unsigned int k=0;
//    do
//    {
//        HAL_UART_Transmit(&UartHandle2,(uint8_t *)(str + k) ,1,1000);
//        k++;
//    } while(*(str + k)!='\0');
//}
//
/////重定向c库函数printf到串口DEBUG_USART，重定向后可使用printf函数
//int fputc(int ch, FILE *f)
//{
//    /* 发送一个字节数据到串口DEBUG_USART */
//    HAL_UART_Transmit(&UartHandle2, (uint8_t *)&ch, 1, 1000);
//
//    return (ch);
//}
//
/////重定向c库函数scanf到串口DEBUG_USART，重写向后可使用scanf、getchar等函数
//int fgetc(FILE *f)
//{
//
//    int ch;
//    HAL_UART_Receive(&UartHandle2, (uint8_t *)&ch, 1, 1000);
//    return (ch);
//}
//
///*********************************************END OF FILE**********************/
