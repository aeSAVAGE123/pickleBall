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
#include "bsp_hc05.h"
#include "bsp_usart_blt.h"
#include "bsp_SysTick.h"
#include <stdlib.h>
#include <string.h>
#include "bsp_debug_usart.h"
//#include "protocol.h"
#include "bsp_motor_control.h"
#include "bsp_adc.h"
#include "bsp_pid.h"
#include "bsp_usart.h"
#include "retarget.h"
void SystemClock_Config(void);
static void MX_NVIC_Init(void);
BLEDev bleDevList;
unsigned int Task_Delay[NumOfTask];
char sendData[1024];
char linebuff[1024];

_Bool firststart;

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

    char* redata;
    uint16_t len;

    RetargetInit(&UartHandle2);
    firststart = 0;
    __IO uint16_t motor1_ChannelPulse = 1500; // 9000
    __IO uint16_t motor2_ChannelPulse = 1500;
    __IO uint16_t motor3_ChannelPulse = PWM_MAX_PERIOD_COUNT/10 + PWM_MAX_PERIOD_COUNT/10 + PWM_MAX_PERIOD_COUNT/10;    //9000 x 3/10 = 2700
    __IO uint16_t motor4_ChannelPulse = PWM_MAX_PERIOD_COUNT/10 + PWM_MAX_PERIOD_COUNT/10 + PWM_MAX_PERIOD_COUNT/10;    //9000 x 3/10 = 2700
//	__IO uint16_t motor5_ChannelPulse = PWM_MAX_PERIOD_COUNT/10 + PWM_MAX_PERIOD_COUNT/10 + PWM_MAX_PERIOD_COUNT/10;    //5500 x 3/10 = 550 x 3 = 1650

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
//    USART_Config();
    DEBUG_USART_Config();
    MX_NVIC_Init();

    BLE_INFO("**********BLEModel EXperment************");

    if(BLE_Init() == 0) {
        BLE_INFO("BLE test normal");
    }
    else
    {
        BLE_ERROR("BLE error");
    }

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
//        receiving_process();
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
            LED5_TOGGLE
            BLE_SendString("1");
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
            BLE_SendString("2");
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
            BLE_SendString("3");
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
            BLE_SendString("4");
        }
        /* 扫描KEY5 */
        if( Key_Scan(KEY5_GPIO_PORT, KEY5_PIN) == KEY_ON)
        {
            if(!is_motor1_en && !is_motor2_en)
            {
                set_motor1_enable();
                set_motor1_direction(MOTOR_FWD);
                set_motor1_speed(motor1_ChannelPulse);

                set_motor2_enable();
                set_motor2_direction(MOTOR_REV);
                set_motor2_speed(motor2_ChannelPulse);
            }
            else
            {
                set_motor1_disable();
                set_motor2_disable();
            }
            LED5_TOGGLE
            BLE_SendString("5");
        }
        if(IS_BLE_CONNECTED())
        {
            BLE_WAKEUP_LOW;
//            if (Task_Delay[0] == 0)
//            {
                uint16_t linelen;
                /*获取数据*/
                redata = get_rebuff(&len);
                linelen = get_line(linebuff, redata, len);
                /*检查数据是否有更新*/
                if (linelen < 200 && linelen != 0)
                {
                    if (strcmp(redata, "1") == 0)
                    {
                        LED5_TOGGLE
                        clean_rebuff();
//                        BLE_SendString("1");
                        uint16_t temp_val = get_pid_target(&pid4); // 获取当前PID目标值
                        if (temp_val >= 700) {
                            temp_val -= 150; // 每次减少150
                                if (temp_val < 700)
                                {
                                    temp_val = 700; // 如果减少后的值小于700，则将其设置为700
                                }
                                set_pid_target4(&pid4, temp_val); // 设置新的PID目标值
                            }
                    }
                    else if (strcmp(redata, "2") == 0)
                    {
                        LED2_TOGGLE
                        clean_rebuff();
//                        BLE_SendString("2");
                        uint16_t temp_val = get_pid_target(&pid4); // 获取当前PID目标值
                        if (temp_val <= 2100) {
                            temp_val += 150; // 每次增加150
                            if (temp_val > 2100) {
                                temp_val = 2100; // 如果增加后的值大于2100，则将其设置为2100
                            }
                            set_pid_target4(&pid4, temp_val); // 设置新的PID目标值
                        }
                    }
                    else if (strcmp(redata, "3") == 0)
                    {
                        LED3_TOGGLE
                        clean_rebuff();
//                        BLE_SendString("3");
                        uint16_t temp_val = get_pid_target(&pid3); // 获取当前PID目标值
                        if(temp_val >= 700)
                        {
                            temp_val -= 150; // 每次减少150
                            if(temp_val < 700) {
                                temp_val = 700; // 如果减少后的值小于700，则将其设置为700
                            }
                            set_pid_target3(&pid3, temp_val); // 设置新的PID目标值
                        }
                    }
                    else if (strcmp(redata, "4") == 0)
                    {
                        LED4_TOGGLE
                        clean_rebuff();
//                        BLE_SendString("4");
                        uint16_t temp_val = get_pid_target(&pid3); // 获取当前PID目标值
                        if(temp_val <= 2200)
                        {
                            temp_val += 150; // 每次增加100
                            if(temp_val > 2200) {
                                temp_val = 2200; // 如果增加后的值大于2100，则将其设置为2100
                            }
                            set_pid_target3(&pid3, temp_val); // 设置新的PID目标值
                        }
                    }
                    else if (strcmp(redata, "5") == 0)
                    {
                        LED5_TOGGLE
                        clean_rebuff();
//                        BLE_SendString("5");
                        if(!is_motor1_en && !is_motor2_en)
                        {
                            set_motor1_enable();
                            set_motor1_direction(MOTOR_FWD);
                            set_motor1_speed(motor1_ChannelPulse);

                            set_motor2_enable();
                            set_motor2_direction(MOTOR_REV);
                            set_motor2_speed(motor2_ChannelPulse);
                        }
                        else
                        {
                            set_motor1_disable();
                            set_motor2_disable();
                        }
                    }
                    else if (strcmp(redata, "6") == 0)
                    {
                        LED5_TOGGLE
                        clean_rebuff();
//                        BLE_SendString("6");
                        motor1_ChannelPulse += 500;
                        motor2_ChannelPulse += 500;
                        if((motor1_ChannelPulse > PWM_MAX_PERIOD_COUNT)||(motor2_ChannelPulse > PWM_MAX_PERIOD_COUNT))
                        {
                            motor1_ChannelPulse = PWM_MAX_PERIOD_COUNT;
                            motor2_ChannelPulse = PWM_MAX_PERIOD_COUNT;
                        }
                        set_motor1_speed(motor1_ChannelPulse);
                        set_motor2_speed(motor2_ChannelPulse);
                    }
                    else if (strcmp(redata, "7") == 0)
                    {
                        LED5_TOGGLE
                        clean_rebuff();
//                        BLE_SendString("7");
                        motor1_ChannelPulse -= 500;
                        motor2_ChannelPulse -= 500;
                        if((motor1_ChannelPulse < PWM_MAX_PERIOD_COUNT/10)||(motor2_ChannelPulse < PWM_MAX_PERIOD_COUNT/10))
                        {
                            motor1_ChannelPulse = 0;
                            motor2_ChannelPulse = 0;
                        }

                        set_motor1_speed(motor1_ChannelPulse);
                        set_motor2_speed(motor2_ChannelPulse);
                    }
                    else if (strcmp(redata, "8") == 0)
                    {
                        LED5_TOGGLE
                        clean_rebuff();
                        __set_FAULTMASK(1);
                        NVIC_SystemReset();
                    }
                    else if (strcmp(redata, "9") == 0)
                    {
                        LED5_TOGGLE
                        clean_rebuff();
                        if(!is_motor5_en)
                        {
                            set_motor5_enable();
                            set_motor5_direction(MOTOR_FWD);
                            set_motor5_speed(3000);
                        }
                        else
                        {
                            set_motor5_disable();
                        }
                    }
                    else {
                        BLE_INFO("receive:\r\n%s", redata);
                    }

                    /*处理数据后，清空接收蓝牙模块数据的缓冲区*/
                    clean_rebuff();
//                    BLE_WAKEUP_HIGHT;
//                }
//                Task_Delay[0] = 600;//此值每1ms会减1，减到0才可以重新进来这里，所以执行的周期是500ms
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
    HAL_NVIC_SetPriority(DEBUG_USART_IRQ, 6, 0);
    HAL_NVIC_EnableIRQ(DEBUG_USART_IRQ );

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
