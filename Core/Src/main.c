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
unsigned int Task_Delay[NumOfTask];
char linebuff[1024];

typedef struct {
    float horizontal;
    float vertical;
    float M1speed;
    float M2speed;
} Position;

// 初始化定点数据
Position positions[25] = {
        {1100, 2500,2000,2000}, {1100, 1900,2000,2000}, {1100, 1738,2000,2000}, {1100, 1290,2000,2000}, {1100, 800,2000,2000},
        {1100, 2250,2000,2000}, {1100, 1900,2000,2000}, {1100, 1591,2000,2000}, {1100, 745,2000,2000}, {1100, 789,2000,2000},
        {1100, 2000,2000,2000}, {1100, 1750,2000,2000}, {1100, 1580,2000,2000}, {1100, 1200,2000,2000}, {1100, 1000,2000,2000},
        {1100, 2100,2000,2000}, {1100, 1780,2000,2000}, {1100, 1530,2000,2000}, {1100, 1300,2000,2000}, {1100, 1200,2000,2000},
        {1100, 1920,2000,2000}, {1100, 1720,2000,2000}, {1100, 1560,2000,2000}, {1100, 1230,2000,2000}, {1100, 800,2000,2000}
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
    char* redata;
    uint16_t len;
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
        if (Task_Delay[0] == 0 && IS_BLE_CONNECTED()) {
            BLE_WAKEUP_LOW;
            uint16_t linelen;
            /*获取数据*/
            redata = get_rebuff(&len);
            linelen = get_line(linebuff, redata, len);
            /*检查数据是否有更新*/
            if (linelen < 200 && linelen != 0) {
                char first_char = redata[0];
                char second_char[3];
                second_char[0] = redata[1];
                second_char[1] = redata[2];
                second_char[2] = '\0';  // 添加字符串结尾
                clean_rebuff();
                if (first_char == 'a')    //向上
                {
                    clean_rebuff();
                    LED5_TOGGLE
                    uint16_t temp_val = get_pid_target(&pid4); // 获取当前PID目标值
                    if (temp_val >= 700) {
                        temp_val -= 150; // 每次减少150
                        if (temp_val < 700) {
                            temp_val = 700; // 如果减少后的值小于700，则将其设置为700
                        }
                        set_pid_target4(&pid4, temp_val); // 设置新的PID目标值
                    }
                }
                if (first_char == 'b')       //向下
                {
                    clean_rebuff();
                    LED2_TOGGLE
                    uint16_t temp_val = get_pid_target(&pid4); // 获取当前PID目标值
                    if (temp_val <= 2100) {
                        temp_val += 150; // 每次增加150
                        if (temp_val > 2100) {
                            temp_val = 2100; // 如果增加后的值大于2100，则将其设置为2100
                        }
                        set_pid_target4(&pid4, temp_val); // 设置新的PID目标值
                    }
                }
                if (first_char == '3')          //左
                {
                    clean_rebuff();
                    LED3_TOGGLE
                    uint16_t temp_val = get_pid_target(&pid3); // 获取当前PID目标值
                    if (temp_val >= 700) {
                        temp_val -= 150; // 每次减少150
                        if (temp_val < 700) {
                            temp_val = 700; // 如果减少后的值小于700，则将其设置为700
                        }
                        set_pid_target3(&pid3, temp_val); // 设置新的PID目标值
                    }
                }
                if (first_char == '4')       //右
                {
                    clean_rebuff();
                    LED4_TOGGLE
                    uint16_t temp_val = get_pid_target(&pid3); // 获取当前PID目标值
                    if (temp_val <= 2200) {
                        temp_val += 150; // 每次增加100
                        if (temp_val > 2200) {
                            temp_val = 2200; // 如果增加后的值大于2100，则将其设置为2100
                        }
                        set_pid_target3(&pid3, temp_val); // 设置新的PID目标值
                    }
                }
                if (first_char == '5')       //启停M1、M2
                {
                    clean_rebuff();
                    LED5_TOGGLE
                    if (!is_motor1_en && !is_motor2_en) {
                        set_motor1_enable();
                        set_motor1_direction(MOTOR_FWD);
                        set_motor1_speed(1800);

                        set_motor2_enable();
                        set_motor2_direction(MOTOR_REV);
                        set_motor2_speed(1800);
                    }
                    else {
                        set_motor1_disable();
                        set_motor2_disable();
                    }
                }
                if (first_char == 'c')       //设置M1、M2不同占空比
                {
                    clean_rebuff();
                    LED5_TOGGLE
                    if (!is_motor1_en && !is_motor2_en)
                    {
                        set_motor1_enable();
                        set_motor1_direction(MOTOR_FWD);
                        set_motor1_speed(1500);

                        set_motor2_enable();
                        set_motor2_direction(MOTOR_REV);
                        set_motor2_speed(2500);
                    }
                    else
                    {
                        set_motor1_disable();
                        set_motor2_disable();
                    }
                }
                if (first_char == 'd')       //设置M1、M2不同占空比
                {
                    clean_rebuff();
                    LED5_TOGGLE
                    if (!is_motor1_en && !is_motor2_en) {
                        set_motor1_enable();
                        set_motor1_direction(MOTOR_FWD);
                        set_motor1_speed(2500);

                        set_motor2_enable();
                        set_motor2_direction(MOTOR_REV);
                        set_motor2_speed(1500);
                    }
                    else {
                        set_motor1_disable();
                        set_motor2_disable();
                    }
                }
                if (first_char == '6')       //M1、M2加速
                {
                    clean_rebuff();
                    __IO uint16_t motor1_ChannelPulse = 2000; // 9000
                    __IO uint16_t motor2_ChannelPulse = 2000;

                    LED5_TOGGLE
                    motor1_ChannelPulse += 100;
                    motor2_ChannelPulse += 100;
                    if ((motor1_ChannelPulse > PWM_MAX_PERIOD_COUNT) || (motor2_ChannelPulse > PWM_MAX_PERIOD_COUNT)) {
                        motor1_ChannelPulse = PWM_MAX_PERIOD_COUNT;
                        motor2_ChannelPulse = PWM_MAX_PERIOD_COUNT;
                    }
                    set_motor1_speed(motor1_ChannelPulse);
                    set_motor2_speed(motor2_ChannelPulse);
                }
                if (first_char == '7')           //M1、M2减速
                {
                    clean_rebuff();
                    __IO uint16_t motor1_ChannelPulse = 2000; // 9000
                    __IO uint16_t motor2_ChannelPulse = 2000;
                    LED5_TOGGLE
                    motor1_ChannelPulse -= 100;
                    motor2_ChannelPulse -= 100;
                    if ((motor1_ChannelPulse < PWM_MAX_PERIOD_COUNT / 10) ||
                        (motor2_ChannelPulse < PWM_MAX_PERIOD_COUNT / 10)) {
                        motor1_ChannelPulse = 0;
                        motor2_ChannelPulse = 0;
                    }

                    set_motor1_speed(motor1_ChannelPulse);
                    set_motor2_speed(motor2_ChannelPulse);
                }
                if (first_char == '8')           //启停M5
                {
                    clean_rebuff();
                    LED5_TOGGLE
                    if (!is_motor5_en) {
                        set_motor5_enable();
                        set_motor5_direction(MOTOR_FWD);
                        set_motor5_speed(3000);
                    } else {
                        set_motor5_disable();
                    }
                }
                if (first_char == '9')           //复位
                {
                    clean_rebuff();
                    LED5_TOGGLE
                    __set_FAULTMASK(1);
                    NVIC_SystemReset();
                }
                switch (first_char)
                {
                    case '1':
                        if(strcmp(second_char, "A1") == 0)
                        {
                            clean_rebuff();
                            LED1_TOGGLE
                            A1_control();
                        }
                        if(strcmp(second_char, "A2") == 0)
                        {
                            clean_rebuff();
                            LED2_TOGGLE
                            A2_control();
                        }
                        if(strcmp(second_char, "A3") == 0)
                        {
                            clean_rebuff();
                            LED3_TOGGLE
                            A3_control();
                        }
                        if(strcmp(second_char, "A4") == 0)
                        {
                            clean_rebuff();
                            LED4_TOGGLE
                            A4_control();
                        }
                        if(strcmp(second_char, "A5") == 0)
                        {
                            clean_rebuff();
                            LED5_TOGGLE
                            A5_control();
                        }
                        if(strcmp(second_char, "B1") == 0)
                        {
                            clean_rebuff();
                            LED1_TOGGLE
                            B1_control();
                        }
                        if(strcmp(second_char, "B2") == 0)
                        {
                            clean_rebuff();
                            LED2_TOGGLE
                            B2_control();
                        }
                        if(strcmp(second_char, "B3") == 0)
                        {
                            clean_rebuff();
                            LED3_TOGGLE
                            B3_control();
                        }
                        if(strcmp(second_char, "B4") == 0)
                        {
                            clean_rebuff();
                            LED4_TOGGLE
                            B4_control();
                        }
                        if(strcmp(second_char, "B5") == 0)
                        {
                            clean_rebuff();
                            LED5_TOGGLE
                            B5_control();
                        }
                        if(strcmp(second_char, "C1") == 0)
                        {
                            clean_rebuff();
                            LED1_TOGGLE
                            C1_control();
                        }
                        if(strcmp(second_char, "C2") == 0)
                        {
                            clean_rebuff();
                            LED2_TOGGLE
                            C2_control();
                        }
                        if(strcmp(second_char, "C3") == 0)
                        {
                            clean_rebuff();
                            LED3_TOGGLE
                            C3_control();
                        }
                        if(strcmp(second_char, "C4") == 0)
                        {
                            clean_rebuff();
                            LED4_TOGGLE
                            C4_control();
                        }
                        if(strcmp(second_char, "C5") == 0)
                        {
                            clean_rebuff();
                            LED5_TOGGLE
                            C5_control();
                        }
                        if(strcmp(second_char, "D1") == 0)
                        {
                            clean_rebuff();
                            LED1_TOGGLE
                            D1_control();
                        }
                        if(strcmp(second_char, "D2") == 0)
                        {
                            clean_rebuff();
                            LED2_TOGGLE
                            D2_control();
                        }
                        if(strcmp(second_char, "D3") == 0)
                        {
                            clean_rebuff();
                            LED3_TOGGLE
                            D3_control();
                        }
                        if(strcmp(second_char, "D4") == 0)
                        {
                            clean_rebuff();
                            LED4_TOGGLE
                            D4_control();
                        }
                        if(strcmp(second_char, "D5") == 0)
                        {
                            clean_rebuff();
                            LED5_TOGGLE
                            D5_control();
                        }
                        if(strcmp(second_char, "E1") == 0)
                        {
                            clean_rebuff();
                            LED1_TOGGLE
                            E1_control();
                        }
                        if(strcmp(second_char, "E2") == 0)
                        {
                            clean_rebuff();
                            LED2_TOGGLE
                            E2_control();
                        }
                        if(strcmp(second_char, "E3") == 0)
                        {
                            clean_rebuff();
                            LED3_TOGGLE
                            E3_control();
                        }
                        if(strcmp(second_char, "E4") == 0)
                        {
                            clean_rebuff();
                            LED4_TOGGLE
                            E4_control();
                        }
                        if(strcmp(second_char, "E5") == 0)
                        {
                            clean_rebuff();
                            LED5_TOGGLE
                            E5_control();
                        }
                        break;
                    case '2':
                        clean_rebuff();
                        LED4_TOGGLE
                        if(random_flag == 1)
                        {
                            random_flag = 0;
                            set_motor5_disable();
                        }
                        else
                        {
                            random_flag = 1;
                        }
                        break;
                }
                /*处理数据后，清空接收蓝牙模块数据的缓冲区*/
                clean_rebuff();
                Task_Delay[0] = 100;                            //此值每1ms会减1，减到0才可以重新进来这里，所以执行的周期是100ms
            }
            BLE_WAKEUP_HIGHT;
        }
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
                set_motor1_enable();                            //A1
                set_motor1_direction(MOTOR_FWD);
                set_motor1_speed(selected_position.M1speed);
                set_motor2_enable();
                set_motor2_direction(MOTOR_REV);
                set_motor2_speed(selected_position.M2speed);
                HAL_Delay(2000);
/** 两秒过后才打开电机的启动 */
                set_motor5_direction(MOTOR_REV);
                set_motor5_speed(3500);
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
