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

BLTDev bltDevList;
unsigned int Task_Delay[NumOfTask];
char sendData[1024];
char linebuff[1024];

void SystemClock_Config(void);


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

    __IO uint16_t motor1_ChannelPulse = 3000; // 9000
	__IO uint16_t motor2_ChannelPulse = 3000;
	__IO uint16_t motor3_ChannelPulse = PWM_MAX_PERIOD_COUNT/10 + PWM_MAX_PERIOD_COUNT/10 + PWM_MAX_PERIOD_COUNT/10;    //9000 x 3/10 = 2700
	__IO uint16_t motor4_ChannelPulse = PWM_MAX_PERIOD_COUNT/10 + PWM_MAX_PERIOD_COUNT/10 + PWM_MAX_PERIOD_COUNT/10;    //9000 x 3/10 = 2700
//	__IO uint16_t motor5_ChannelPulse = PWM_MAX_PERIOD_COUNT/10 + PWM_MAX_PERIOD_COUNT/10 + PWM_MAX_PERIOD_COUNT/10;    //5500 x 3/10 = 550 x 3 = 1650
    char* redata;
    uint16_t len;
    SysTick_Init();
    unsigned long count;
#if 0
    static uint8_t hc05_role=1;
	char hc05_mode[10]="MASTER";
	char hc05_name[30]="HC05_MASTER";
#else
    static uint8_t hc05_role=0;
    char hc05_mode[10]="SLAVE";
    char hc05_name[30]="HC05_SLAVE";
#endif
    char hc05_nameCMD[40];
    HAL_Init();
    /** 初始化系统时钟为72MHz */
    SystemClock_Config();

    /** LED 灯初始化 */
    LED_GPIO_Config();
    /** ADC初始化 */
    ADC1_Init();
    HC05_Init();
//    /** 协议初始化 */
    //  protocol_init();
//    /** 调试串口初始化 */
    //   DEBUG_USART_Config();
//
    HC05_INFO("**********HC05模块实验************");

    if(HC05_Init() == 0)
        HC05_INFO("HC05模块检测正常。");
    else
    {
        HC05_ERROR("HC05模块检测不正常，请检查模块与开发板的连接，然后复位开发板重新测试。");
        while(1);
    }
    /*复位、恢复默认状态*/
    HC05_Send_CMD("AT+RESET\r\n",1);
    HAL_Delay(800);

    HC05_Send_CMD("AT+ORGL\r\n",1);
    HAL_Delay(200);


    /*各种命令测试演示，默认不显示。
     *在bsp_hc05.h文件把HC05_DEBUG_ON 宏设置为1，
     *即可通过串口调试助手接收调试信息*/

    HC05_Send_CMD("AT+VERSION?\r\n",1);

    HC05_Send_CMD("AT+ADDR?\r\n",1);

    HC05_Send_CMD("AT+UART?\r\n",1);

    HC05_Send_CMD("AT+CMODE?\r\n",1);

    HC05_Send_CMD("AT+STATE?\r\n",1);

    // HC05_Send_CMD("AT+ROLE=1\r\n",1);
    HC05_Send_CMD("AT+ROLE=0\r\n",1);

    /*初始化SPP规范*/
    HC05_Send_CMD("AT+INIT\r\n",1);
    HC05_Send_CMD("AT+CLASS=0\r\n",1);
    HC05_Send_CMD("AT+INQM=1,9,48\r\n",1);

    /*设置模块名字*/
    sprintf(hc05_nameCMD,"AT+NAME=%s\r\n",hc05_name);
    HC05_Send_CMD(hc05_nameCMD,1);

    HC05_INFO("本模块名字为:%s ,模块已准备就绪。",hc05_name);

    /** 初始化按键GPIO */
    Key_GPIO_Config();
    /** 电机初始化 */
    TIMx_Configuration();
    /** 算法执行定时器 */
    Basic_TIMx_Configuration();
    /** 对PID参数进行初始化 */
	PID_param_init();


   set_p_i_d(&pid3, 25.0, 0.17, 0.0);
   set_p_i_d(&pid4, 25.0,0.19, 0.0);//增量式

    float pid_temp3[3] = {pid3.Kp, pid3.Ki, pid3.Kd};
//	float pid_temp4[3] = {pid4.Kp, pid4.Ki, pid4.Kd};
//	set_computer_value(SEND_P_I_D_CMD, CURVES_CH1, pid_temp3, 3);

    /** 对上、下电机进行归零操作 */
	pid3.actual_val = positiondown_adc_mean;
	pid4.actual_val = positionup_adc_mean;
	set_pid_target(&pid3, 1618);
	set_pid_target(&pid4, 1290);

    printf("野火电机开发/r/n");
    wakeup_motor();



    while (1)
    {
        while (!IS_HC05_CONNECTED()) {
            if(hc05_role == 1)	//主模式
            {
                HC05_INFO("正在扫描蓝牙设备...");

                linkHC05();

            }
            else	//从模式
            {
                HC05_Send_CMD("AT+INQ\r\n",1);//模块在查询状态，才能容易被其它设备搜索到
                HAL_Delay(1000);
                HC05_Send_CMD("AT+INQC\r\n",1);//中断查询，防止查询的结果干扰串口透传的接收
            }
//            LED1_TOGGLE;
        }

        /* 扫描KEY1 */
        if( Key_Scan(KEY1_GPIO_PORT, KEY1_PIN) == KEY_ON)
        {
            if(!is_motor1_en && !is_motor2_en)
            {
                set_motor1_enable();
                set_motor1_direction(MOTOR_FWD);
                set_motor1_speed(1800);

                set_motor2_enable();
                set_motor1_direction(MOTOR_REV);
                set_motor2_speed(1800);
            }
            else
            {
                set_motor1_disable();
                set_motor2_disable();
            }
            HC05_SendString("1");
        }
        /* 扫描KEY2 */
        if( Key_Scan(KEY2_GPIO_PORT, KEY2_PIN) == KEY_ON)
        {
            if(!is_motor3_en)
            {
                set_motor3_enable();
//                set_motor3_direction(MOTOR_REV);
//                set_motor3_speed(3000);
            }
            else
            {
                set_motor3_disable();
            }
            LED2_TOGGLE
            HC05_SendString("2");
        }

        /* 扫描KEY3 */
        if( Key_Scan(KEY3_GPIO_PORT, KEY3_PIN) == KEY_ON)
        {
            LED3_TOGGLE
            if(!is_motor4_en)
            {
                set_motor4_enable();
//                set_motor4_direction(MOTOR_REV);
//                set_motor4_speed(3000);
            }
            else
            {
                set_motor4_disable();
            }
            HC05_SendString("3");
        }

        /* 扫描KEY4 */
        if( Key_Scan(KEY4_GPIO_PORT, KEY4_PIN) == KEY_ON)
        {
            LED4_TOGGLE
            if(MOTOR3_direction == MOTOR_FWD)
            {
                set_motor3_direction(MOTOR_REV);
            }
            else
            {
                set_motor3_direction(MOTOR_FWD);
            }
            HC05_SendString("4");
        }
        /* 扫描KEY5 */
        if( Key_Scan(KEY5_GPIO_PORT, KEY5_PIN) == KEY_ON)
        {
            set_motor1_disable();
            set_motor2_disable();
            LED5_TOGGLE
            HC05_SendString("5");
        }
        //连接后每隔一段时间检查接收缓冲区
        if(Task_Delay[0]==0 && IS_HC05_CONNECTED())
        {
            uint16_t linelen;

            /*获取数据*/
            redata = get_rebuff(&len);
            linelen = get_line(linebuff,redata,len);

            /*检查数据是否有更新*/
            if(linelen<200 && linelen != 0)
            {
                if(strcmp(redata,"2")==0) {
                    LED2_TOGGLE;
                    if(!is_motor1_en && !is_motor2_en)
                    {
                        set_motor1_enable();
                        set_motor1_direction(MOTOR_FWD);
                        set_motor1_speed(1800);

                        set_motor2_enable();
                        set_motor1_direction(MOTOR_REV);
                        set_motor2_speed(1800);
                    }
                    else
                    {
                        set_motor1_disable();
                        set_motor2_disable();
                    }
                    HC05_INFO("receive:\r\n%s",redata);
                } else if(strcmp(redata,"3")==0) {
                    LED3_TOGGLE;
                    HC05_INFO("receive:\r\n%s",redata);
                } else if(strcmp(redata,"4")==0) {
                    LED4_TOGGLE;
                    HC05_INFO("receive:\r\n%s",redata);
                } else if(strcmp(redata,"5")==0) {
                    LED5_TOGGLE;
                    set_motor1_disable();
                    set_motor2_disable();
                    HC05_INFO("receive:\r\n%s",redata);
                } else {
                    HC05_INFO("receive:\r\n%s",redata);
                }

                /*处理数据后，清空接收蓝牙模块数据的缓冲区*/
                clean_rebuff();

            }
            Task_Delay[0]=600;//此值每1ms会减1，减到0才可以重新进来这里，所以执行的周期是500ms
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

