/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "gpio.h"
#include "bsp_led.h"
#include "bsp_key.h"
#include "bsp_tim.h"
#include "bsp_basic_tim.h"
#include "bsp_debug_usart.h"
#include "bsp_usart.h"
#include "protocol.h"
#include "bsp_motor_control.h"
#include "bsp_adc.h"
#include "bsp_pid.h"
#include "bsp_usart_blt.h"
#include "bsp_SysTick.h"

void SystemClock_Config(void);
static void MX_NVIC_Init(void);
unsigned int Task_Delay[NumOfTask];

_Bool firststart;
_Bool speedflag = 0;
_Bool Pid3flag = 0;
_Bool Pid4flag = 0;
int current = 0;
int upcurrent = 0;

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
//    BLE_Init();
    /** LED 灯初始化 */
    LED_GPIO_Config();
    /** ADC初始化 */
    ADC1_Init();
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

    _Bool cnt1 = 1;
    _Bool cnt2 = 1;
    _Bool cnt3 = 1;
    _Bool cnt4 = 1;
    _Bool cnt5 = 1;

    _Bool upcnt1 = 1;
    _Bool upcnt2 = 1;
    _Bool upcnt3 = 1;
    _Bool upcnt4 = 1;
    _Bool upcnt5 = 1;

    while (1)
    {
        Key_control();
        if(speedflag == 1)
        {
            int32_t tempspeed = Rotation1_adc_mean;
            if(tempspeed > 3000)
            {
                tempspeed = 3000;
            }
            set_motor1_speed(tempspeed);
            set_motor2_speed(tempspeed);
        }
        if(Pid3flag == 1)
        {
            current = Rotation2_adc_mean;
            if(current > 0 && current < 819)
            {
                if(cnt1 == 1)
                {
                    set_pid_target3(&pid3,780);
                    cnt1 = 0;
                }
            } else
            {
                cnt1 = 1;
            }
            if(current >= 819 && current < 1638)
            {
                if(cnt2 == 1)
                {
                    set_pid_target3(&pid3,1250);
                    cnt2 = 0;
                }
            } else
            {
                cnt2 = 1;
            }
            if(current >= 1638 && current < 2457)
            {
                if(cnt3 == 1)
                {
                    set_pid_target3(&pid3,1600);
                    cnt3 = 0;
                }
            } else
            {
                cnt3 = 1;
            }
            if(current >= 2457 && current < 3276)
            {
                if(cnt4 == 1)
                {
                    set_pid_target3(&pid3,1800);
                    cnt4 = 0;
                }
            } else
            {
                cnt4 = 1;
            }
            if(current >= 3276 && current < 4095)
            {
                if(cnt5 == 1)
                {
                    set_pid_target3(&pid3,2400);
                    cnt5 = 0;
                }
            } else
            {
                cnt5 = 1;
            }
        }
        if(Pid4flag == 1)
        {
            upcurrent = Rotation3_adc_mean;
            if(upcurrent > 0 && upcurrent < 819)
            {
                if(upcnt1 == 1)
                {
                    set_pid_target4(&pid3,900);
                    upcnt1 = 0;
                }
            } else
            {
                upcnt1 = 1;
            }
            if(upcurrent >= 819 && upcurrent < 1638)
            {
                if(upcnt2 == 1)
                {
                    set_pid_target4(&pid3,1100);
                    upcnt2 = 0;
                }
            } else
            {
                upcnt2 = 1;
            }
            if(upcurrent >= 1638 && upcurrent < 2457)
            {
                if(upcnt3 == 1)
                {
                    set_pid_target4(&pid3,1300);
                    upcnt3 = 0;
                }
            } else
            {
                upcnt3 = 1;
            }
            if(upcurrent >= 2457 && upcurrent < 3276)
            {
                if(upcnt4 == 1)
                {
                    set_pid_target4(&pid3,1500);
                    upcnt4 = 0;
                }
            } else
            {
                upcnt4 = 1;
            }
            if(upcurrent >= 3276 && upcurrent < 4095)
            {
                if(upcnt5 == 1)
                {
                    set_pid_target4(&pid3,1700);
                    upcnt5 = 0;
                }
            } else
            {
                upcnt5 = 1;
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
    HAL_NVIC_SetPriority(USART_IRQ, 4, 0);
    HAL_NVIC_EnableIRQ(USART_IRQ);

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
