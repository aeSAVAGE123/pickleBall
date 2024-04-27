/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "gpio.h"
#include "bsp_led.h"
#include "bsp_key.h"
#include "bsp_tim.h"
#include "bsp_basic_tim.h"
#include "bsp_usart.h"
#include "protocol.h"
#include "bsp_motor_control.h"
#include "bsp_adc.h"
#include "bsp_pid.h"
#include "bsp_SysTick.h"
#include "bsp_hc05.h"
#include <string.h>
#include "bsp_usart_blt.h"

void SystemClock_Config(void);
static void MX_NVIC_Init(void);

_Bool firststart;
_Bool speedflag = 0;
_Bool Pid3flag = 0;
_Bool Pid4flag = 0;
_Bool random_flag = 0;
_Bool randomcnt1 = 1;


typedef struct {
    float horizontal;
    float vertical;
} Position;

// 初始化定点数据
Position positions[25] = {
        {1100, 2500}, {1100, 1900}, {1100, 1738}, {1100, 1290}, {1100, 800},
        {1100, 2250}, {1100, 1900}, {1100, 1591}, {1100, 745}, {1100, 789},
        {1100, 2000}, {1100, 1750}, {1100, 1580}, {1100, 1200}, {1100, 1000},
        {1100, 2100}, {1100, 1780}, {1100, 1530}, {1100, 1300}, {1100, 1200},
        {1100, 1920}, {1100, 1720}, {1100, 1560}, {1100, 1230}, {1100, 800}
};

// 函数来生成指定范围内的随机数
int generate_random_position() {
    return rand() % 25; // 返回0到24之间的随机数
}

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
    firststart = 0;
    HAL_Init();
    /** 初始化系统时钟为72MHz */
    SystemClock_Config();
    BLE_Init();
    /** LED 灯初始化 */
    LED_GPIO_Config();
    /** ADC初始化 */
    ADC_Init();
    /** 初始化按键GPIO */
    Key_GPIO_Config();
    /** 电机初始化 */
    TIMx_Configuration();
    /** 算法执行定时器 */
    Basic_TIMx_Configuration();
    /** 对PID参数进行初始化 */
    PID_param_init();
    USART_Config();
    MX_NVIC_Init();


    set_p_i_d(&pid3, 25, 0.18, 0.0);
    set_p_i_d(&pid4, 25.0,0.19, 0.0);//增量式


    /** 对上、下电机进行归零操作 */
    pid3.actual_val = positiondown_adc_mean;
    pid4.actual_val = positionup_adc_mean;
    set_pid_target3(&pid3, 1618);
    set_pid_target4(&pid4, 1290);

    wakeup_motor();
    set_motor3_enable();
    set_motor4_enable();

    while (1)
    {
        Key_control();
        Knob_control();
        BLE_motor_control();
        if(1 == random_flag)
        {
             if(Dropping_adc_mean < 400)
             {
                 HAL_Delay(500);
                 randomcnt1 = 1;
                 set_motor5_disable();
             }
/** 保证每个位置执行一次，要等待上方传感器产生低电平信号 */
            if(1 == randomcnt1)
            {
                randomcnt1 = 0;
                srand(HAL_GetTick()); // 初始化随机数发生器
                int index = generate_random_position(); // 获取随机位置索引
                Position selected_position = positions[index]; // 获取选定的位置数据

                set_pid_target3(&pid3, selected_position.vertical);
                set_pid_target4(&pid4, selected_position.horizontal);
                HAL_Delay(2000);
/** 两秒过后才打开电机的启动 */
                set_motor5_direction(MOTOR_REV);
                set_motor5_speed(3000);
                set_motor5_enable();
            }
        }
    }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                  |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */
static void MX_NVIC_Init(void)
{
//    HAL_NVIC_SetPriority(USART_IRQ, 4, 0);
//    HAL_NVIC_EnableIRQ(USART_IRQ);

    HAL_NVIC_SetPriority(ADC_DMA_IRQ, 1, 0);
    HAL_NVIC_EnableIRQ(ADC_DMA_IRQ);

    HAL_NVIC_SetPriority(BASIC_TIM_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(BASIC_TIM_IRQn);

    HAL_NVIC_SetPriority(BASIC_TIM5_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(BASIC_TIM5_IRQn);

    HAL_NVIC_SetPriority(BLE_UARTx_IRQ, 0, 0);
    HAL_NVIC_EnableIRQ(BLE_UARTx_IRQ);

}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1)
    {
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
