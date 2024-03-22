/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "gpio.h"
#include "bsp_led.h"
#include "bsp_key.h"
#include "bsp_tim.h"
#include "bsp_basic_tim.h"
//#include "bsp_debug_usart.h"
//#include "protocol.h"
#include "bsp_motor_control.h"
#include "bsp_adc.h"
#include "bsp_pid.h"
#include "bsp_usart.h"
void SystemClock_Config(void);

_Bool firststart;

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
    firststart = 0;
    __IO uint16_t motor1_ChannelPulse = 3000; // 9000
    __IO uint16_t motor2_ChannelPulse = 3000;
    __IO uint16_t motor3_ChannelPulse = PWM_MAX_PERIOD_COUNT/10 + PWM_MAX_PERIOD_COUNT/10 + PWM_MAX_PERIOD_COUNT/10;    //9000 x 3/10 = 2700
    __IO uint16_t motor4_ChannelPulse = PWM_MAX_PERIOD_COUNT/10 + PWM_MAX_PERIOD_COUNT/10 + PWM_MAX_PERIOD_COUNT/10;    //9000 x 3/10 = 2700
//	__IO uint16_t motor5_ChannelPulse = PWM_MAX_PERIOD_COUNT/10 + PWM_MAX_PERIOD_COUNT/10 + PWM_MAX_PERIOD_COUNT/10;    //5500 x 3/10 = 550 x 3 = 1650

    HAL_Init();
    /** 初始化系统时钟为72MHz */
    SystemClock_Config();
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

    set_p_i_d(&pid3, 25, 0.18, 0.0);
    set_p_i_d(&pid4, 25.0,0.19, 0.0);//增量式

    float pid_temp3[3] = {pid3.Kp, pid3.Ki, pid3.Kd};
	float pid_temp4[3] = {pid4.Kp, pid4.Ki, pid4.Kd};
    float target = 100;


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
        receiving_process();
        /* 扫描KEY1 */
        if( Key_Scan(KEY1_GPIO_PORT, KEY1_PIN) == KEY_ON)
        {
            uint16_t temp_val = get_pid_target(&pid4); // 获取当前PID目标值
            if (temp_val >= 700) {
                temp_val -= 150; // 每次减少150
                if (temp_val < 700) {
                    temp_val = 700; // 如果减少后的值小于700，则将其设置为700
                }
                set_pid_target4(&pid4, temp_val); // 设置新的PID目标值
            }
            LED1_TOGGLE
        }

        /* 扫描KEY2 */
        if( Key_Scan(KEY2_GPIO_PORT, KEY2_PIN) == KEY_ON)
        {
            uint16_t temp_val = get_pid_target(&pid4); // 获取当前PID目标值
            if (temp_val <= 2100) {
                temp_val += 150; // 每次增加150
                if (temp_val > 2100) {
                    temp_val = 2100; // 如果增加后的值大于2100，则将其设置为2100
                }
                set_pid_target4(&pid4, temp_val); // 设置新的PID目标值
            }
            LED2_TOGGLE
        }


            /* 扫描KEY3 */
        if( Key_Scan(KEY3_GPIO_PORT, KEY3_PIN) == KEY_ON)
        {
            uint16_t temp_val = get_pid_target(&pid3); // 获取当前PID目标值
            if(temp_val >= 700)
            {
                temp_val -= 150; // 每次减少150
                if(temp_val < 700) {
                    temp_val = 700; // 如果减少后的值小于700，则将其设置为700
                }
                set_pid_target3(&pid3, temp_val); // 设置新的PID目标值
            }
            LED3_TOGGLE
        }

        /* 扫描KEY4 */
        if( Key_Scan(KEY4_GPIO_PORT, KEY4_PIN) == KEY_ON)
        {
            uint16_t temp_val = get_pid_target(&pid3); // 获取当前PID目标值
            if(temp_val <= 2200)
            {
                temp_val += 150; // 每次增加100
                if(temp_val > 2200) {
                    temp_val = 2200; // 如果增加后的值大于2100，则将其设置为2100
                }
                set_pid_target3(&pid3, temp_val); // 设置新的PID目标值
            }
            LED4_TOGGLE
        }
        /* 扫描KEY5 */
        if( Key_Scan(KEY5_GPIO_PORT, KEY5_PIN) == KEY_ON)
        {
            if(!is_motor1_en && !is_motor2_en)
            {
                set_motor1_enable();
                set_motor1_direction(MOTOR_FWD);
                set_motor1_speed(2400);

                set_motor2_enable();
                set_motor2_direction(MOTOR_REV);
                set_motor2_speed(2400);
            }
            else
            {
                set_motor1_disable();
                set_motor2_disable();
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
