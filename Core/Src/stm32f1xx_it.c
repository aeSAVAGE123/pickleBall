/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_it.h"
#include "bsp_basic_tim.h"
#include "bsp_led.h"
//#include "bsp_debug_usart.h"
//#include "protocol.h"
#include "bsp_adc.h"
#include "bsp_pid.h"
#include "bsp_usart.h"
#include "bsp_usart_blt.h"
#include "bsp_SysTick.h"

extern unsigned char UART_RxPtr;
extern unsigned int Task_Delay[];
extern void TimingDelay_Decrement(void);
extern void TimeStamp_Increment(void);

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */

/* USER CODE END EV */


/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */
    unsigned char i;
    TimingDelay_Decrement();
    TimeStamp_Increment();

    for(i=0;i<NumOfTask;i++)
    {
        if(Task_Delay[i])
        {
            Task_Delay[i]--;
        }
    }
  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/
static uint16_t adc_buff_down[10000];
static uint16_t adc_buff_up[10000];
uint8_t i = 0;
/**
  * @brief  基本定时器中断服务函数----PID计算使用
  * @param  None
  * @retval None
  */
void BASIC_TIM_IRQHandler (void)
{
    HAL_TIM_IRQHandler(&TIM_TimeBaseStructure1);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim==(&TIM_TimeBaseStructure1)) {
        LED1_TOGGLE
//      set_computer_Speed_Location_value(Send_Speed_CMD,positiondown_adc_mean);
        set_computer_Speed_Location_value(Send_Speed_CMD,positionup_adc_mean);
		if (fabs(pid4.err) >= 500 )
		{
                Pflag4 = 1;
				Iflag4 = 0;
				Iflagz4 = 0;
        }
		else if (fabs(pid4.err) >= 50 && fabs(pid4.err) < 500)
		{
                Pflag4 = 1;
				Iflag4 = 1;
				Iflagz4 = 0;
        }
		else if (fabs(pid4.err) < 50)
		{
                Pflag4 = 1;
				Iflag4 = 0;
				Iflagz4 = 1;
        }

        if (fabs(pid3.err) >= 500 )
        {
            Pflag3 = 1;
            Iflag3 = 0;
            Iflagz3 = 0;
        }
        else if (fabs(pid3.err) >= 50 && fabs(pid3.err) < 500)
        {
            Pflag3 = 1;
            Iflag3 = 1;
            Iflagz3 = 0;
        }
        else if (fabs(pid3.err) < 50)
        {
            Pflag3 = 1;
            Iflag3 = 0;
            Iflagz3 = 1;
        }
//			updateFlagsForPID(&pid4,Pflag4,Iflag4,Iflagz4,500,50);
            motor3_pid_control();
            motor4_pid_control();
    }
}

//uint8_t dr;
//volatile uint8_t sr_status;
////接收数组指针
//extern unsigned char UART_RxPtr;
///**
//  * @brief 串口中断服务函数
//  */
//void DEBUG_USART_IRQHandler(void)
//{
//    sr_status = UartHandle.Instance->SR & (1<<3);//clear SR register ORE bit status
//    dr = UartHandle.Instance->DR;
//    protocol_data_recv(&dr, 1);
//    HAL_UART_IRQHandler(&UartHandle);
//}

void ADC_POSITION_IRQHandler(void)
{
    HAL_ADC_IRQHandler(&hadc1);
}

/**
  * @brief  This function handles DMA interrupt request.
  * @param  None
  * @retval None
  */
void ADC_DMA_IRQ_Handler(void)
{
    HAL_DMA_IRQHandler(&DMA_Init_Handle);
}

void USART_IRQHandler(void)
{
    uint8_t data[1];

    data[0] = __HAL_UART_FLUSH_DRREGISTER(&UartHandle);
// data[0] = UartHandle.Instance->DR;
    if(__HAL_UART_GET_IT_SOURCE(&UartHandle, UART_IT_RXNE) != RESET)
    {
        data[0] = UartHandle.Instance->DR;
        PushArr(data_buff,data[0]);
        __HAL_UART_CLEAR_FLAG(&UartHandle, UART_IT_RXNE);
    }
    HAL_UART_IRQHandler(&UartHandle);
    __HAL_UART_ENABLE_IT(&UartHandle,UART_IT_RXNE);
}
void BLT_UARTx_IRQHandler(void)
{
    bsp_USART_Process();
}
