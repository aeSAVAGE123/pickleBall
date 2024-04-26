#include "bsp_motor_control.h"
#include "bsp_debug_usart.h"
#include "bsp_adc.h"
#include "bsp_led.h"
//#include "bsp_pid.h"
#include "bsp_basic_tim.h"
#include "protocol.h"
#include "bsp_tim.h"
#include <math.h>
#include <stdlib.h>
#include "bsp_hc05.h"
#include "bsp_usart_blt.h"
#include <string.h>
#define NumOfTask 3
unsigned int Task_Delay[NumOfTask];
char linebuff[1024];

motor_dir_t MOTOR1_direction  = MOTOR_REV;       // 记录方向
motor_dir_t MOTOR2_direction  = MOTOR_FWD;       // 记录方向
motor_dir_t MOTOR3_direction  = MOTOR_REV;       // 记录方向
motor_dir_t MOTOR4_direction  = MOTOR_FWD;       // 记录方向
motor_dir_t MOTOR5_direction  = MOTOR_FWD;       // 记录方向

uint16_t    motor1_dutyfactor = T1_PEM_motor1_dutyfactor;            // 记录电机1占空比
uint16_t    motor2_dutyfactor = T1_PEM_motor1_dutyfactor;            // 记录电机2占空比
uint16_t    motor3_dutyfactor = T1_PEM_motor1_dutyfactor;            // 记录电机3占空比
uint16_t    motor4_dutyfactor = T1_PEM_motor1_dutyfactor;            // 记录电机4占空比
uint16_t    motor5_dutyfactor = T1_PEM_motor1_dutyfactor;            // 记录电机5占空比

uint8_t     is_motor1_en = 0;            			// 电机1使能
uint8_t     is_motor2_en = 0;            			// 电机2使能
uint8_t     is_motor3_en = 0;            			// 电机3使能
uint8_t     is_motor4_en = 0;            			// 电机4使能
uint8_t     is_motor5_en = 0;            			// 电机5使能

void wakeup_motor(void)
{
    MOTOR_ENABLE_nSLEEP();
}

void sleep_motor(void)
{
    MOTOR_DISABLE_nSLEEP();
}

/**
  * @brief  设置电机1速度
  * @param  v: 速度（占空比）
  * @retval 无
  */
void set_motor1_speed(uint16_t v)
{
    motor1_dutyfactor = v;

    if (MOTOR1_direction == MOTOR_FWD)
    {
        MOTOR1_SET_FWD_COMPAER(motor1_dutyfactor);     // 设置速度
        MOTOR1_REV_DISABLE();
        MOTOR1_FWD_ENABLE();
    }
    else
    {
        MOTOR1_SET_REV_COMPAER(motor1_dutyfactor);     // 设置速度
        MOTOR1_FWD_DISABLE();
        MOTOR1_REV_ENABLE();
    }
}

/**
  * @brief  设置电机1方向
  * @param  无
  * @retval 无
  */
void set_motor1_direction(motor_dir_t dir)
{
    MOTOR1_direction = dir;

    if (MOTOR1_direction == MOTOR_FWD)
    {
        MOTOR1_SET_FWD_COMPAER(motor1_dutyfactor);      // 设置正向速度
        MOTOR1_REV_DISABLE();                           // 设置反向速度
        MOTOR1_FWD_ENABLE();
    }
    else
    {
        MOTOR1_FWD_DISABLE();                           // 设置正向速度
        MOTOR1_SET_REV_COMPAER(motor1_dutyfactor);      // 设置反向速度
        MOTOR1_REV_ENABLE();
    }
}

/**
  * @brief  使能电机1
  * @param  方向
  * @retval 无
  */
void set_motor1_enable(void)
{
    is_motor1_en = 1;
    MOTOR1_FWD_ENABLE();
    MOTOR1_REV_ENABLE();
}

/**
  * @brief  禁用电机1
  * @param  无
  * @retval 无
  */
void set_motor1_disable(void)
{
    is_motor1_en = 0;
    MOTOR1_FWD_DISABLE();
    MOTOR1_REV_DISABLE();
}

/**
  * @brief  设置电机2速度
  * @param  v: 速度（占空比）
  * @retval 无
  */
void set_motor2_speed(uint16_t v)
{
    motor2_dutyfactor = v;

    if (MOTOR2_direction == MOTOR_FWD)
    {
        MOTOR2_SET_FWD_COMPAER(motor2_dutyfactor);     // 设置速度
        MOTOR2_REV_DISABLE();
        MOTOR2_FWD_ENABLE();
    }
    else
    {
        MOTOR2_SET_REV_COMPAER(motor2_dutyfactor);     // 设置速度
        MOTOR2_FWD_DISABLE();
        MOTOR2_REV_ENABLE();
    }
}

/**
  * @brief  设置电机2方向
  * @param  无
  * @retval 无
  */
void set_motor2_direction(motor_dir_t dir)
{
    MOTOR2_direction = dir;

    if (MOTOR2_direction == MOTOR_FWD)
    {
        MOTOR2_SET_FWD_COMPAER(motor2_dutyfactor);      // 设置正向速度
        MOTOR2_REV_DISABLE();
        MOTOR2_FWD_ENABLE();
    }
    else
    {
        MOTOR2_FWD_DISABLE();                           // 设置正向速度
        MOTOR2_SET_REV_COMPAER(motor2_dutyfactor);      // 设置反向速度
        MOTOR2_REV_ENABLE();
    }
}

/**
  * @brief  使能电机2
  * @param  无
  * @retval 无
  */
void set_motor2_enable(void)
{
    is_motor2_en = 1;
    MOTOR2_FWD_ENABLE();
    MOTOR2_REV_ENABLE();
}

/**
  * @brief  禁用电机2
  * @param  无
  * @retval 无
  */
void set_motor2_disable(void)
{
    is_motor2_en = 0;
    MOTOR2_FWD_DISABLE();
    MOTOR2_REV_DISABLE();
}

/**
  * @brief  设置电机3速度
  * @param  v: 速度（占空比）
  * @retval 无
  */
void set_motor3_speed(uint16_t v)
{
    motor3_dutyfactor = v;

    if (MOTOR3_direction == MOTOR_FWD)
    {
        MOTOR3_SET_FWD_COMPAER(motor3_dutyfactor);     // 设置速度
        MOTOR3_REV_DISABLE();
        MOTOR3_FWD_ENABLE();
    }
    else
    {
        MOTOR3_SET_REV_COMPAER(motor3_dutyfactor);     // 设置速度
        MOTOR3_FWD_DISABLE();
        MOTOR3_REV_ENABLE();
    }
}

/**
  * @brief  设置电机3方向
  * @param  无
  * @retval 无
  */
void set_motor3_direction(motor_dir_t dir)
{
    MOTOR3_direction = dir;

    if (MOTOR3_direction == MOTOR_FWD)
    {
        MOTOR3_SET_FWD_COMPAER(motor3_dutyfactor);      // 设置正向速度
        MOTOR3_REV_DISABLE();
        MOTOR3_FWD_ENABLE();
    }
    else
    {
        MOTOR3_FWD_DISABLE();                           // 设置正向速度
        MOTOR3_SET_REV_COMPAER(motor3_dutyfactor);      // 设置反向速度
        MOTOR3_REV_ENABLE();
    }
}

/**
  * @brief  使能电机3
  * @param  无
  * @retval 无
  */
void set_motor3_enable(void)
{
    is_motor3_en = 1;
    MOTOR3_FWD_ENABLE();
    MOTOR3_REV_ENABLE();
}

/**
  * @brief  禁用电机3
  * @param  无
  * @retval 无
  */
void set_motor3_disable(void)
{
    is_motor3_en = 0;
    MOTOR3_FWD_DISABLE();
    MOTOR3_REV_DISABLE();
}

/**
  * @brief  设置电机4速度
  * @param  v: 速度（占空比）
  * @retval 无
  */
void set_motor4_speed(uint16_t v)
{
    motor4_dutyfactor = v;

    if (MOTOR4_direction == MOTOR_FWD)
    {
        MOTOR4_SET_FWD_COMPAER(motor4_dutyfactor);     // 设置速度
        MOTOR4_REV_DISABLE();
        MOTOR4_FWD_ENABLE();
    }
    else
    {
        MOTOR4_SET_REV_COMPAER(motor4_dutyfactor);     // 设置速度
        MOTOR4_FWD_DISABLE();
        MOTOR4_REV_ENABLE();
    }
}

/**
  * @brief  设置电机4方向
  * @param  无
  * @retval 无
  */
void set_motor4_direction(motor_dir_t dir)
{
    MOTOR4_direction = dir;

    if (MOTOR4_direction == MOTOR_FWD)
    {
        MOTOR4_SET_FWD_COMPAER(motor4_dutyfactor);      // 设置正向速度
        MOTOR4_REV_DISABLE();
    }
    else
    {
        MOTOR4_FWD_DISABLE();                           // 设置正向速度
        MOTOR4_SET_REV_COMPAER(motor4_dutyfactor);      // 设置反向速度
    }
}

/**
  * @brief  使能电机4
  * @param  无
  * @retval 无
  */
void set_motor4_enable(void)
{
    is_motor4_en = 1;
    MOTOR4_FWD_ENABLE();
    MOTOR4_REV_ENABLE();
}

/**
  * @brief  禁用电机4
  * @param  无
  * @retval 无
  */
void set_motor4_disable(void)
{
    is_motor4_en = 0;
    MOTOR4_FWD_DISABLE();
    MOTOR4_REV_DISABLE();
}

/**
  * @brief  设置电机5速度
  * @param  v: 速度（占空比）
  * @retval 无
  */
void set_motor5_speed(uint16_t v)
{
    motor5_dutyfactor = v;

    if (MOTOR5_direction == MOTOR_FWD)
    {
        MOTOR5_SET_FWD_COMPAER(motor5_dutyfactor);     // 设置速度
        MOTOR5_REV_DISABLE();
        MOTOR5_FWD_ENABLE();
    }
    else
    {
        MOTOR5_SET_REV_COMPAER(motor5_dutyfactor);     // 设置速度
        MOTOR5_FWD_DISABLE();
        MOTOR5_REV_ENABLE();
    }
}

/**
  * @brief  设置电机5方向
  * @param  无
  * @retval 无
  */
void set_motor5_direction(motor_dir_t dir)
{
    MOTOR5_direction = dir;

    if (MOTOR5_direction == MOTOR_FWD)
    {
        MOTOR5_SET_FWD_COMPAER(motor5_dutyfactor);      // 设置正向速度
        MOTOR5_REV_DISABLE();
        MOTOR5_FWD_ENABLE();
    }
    else
    {
        MOTOR5_FWD_DISABLE();                           // 设置正向速度
        MOTOR5_SET_REV_COMPAER(motor5_dutyfactor);      // 设置反向速度
        MOTOR5_REV_ENABLE();
    }
}

/**
  * @brief  使能电机5
  * @param  无
  * @retval 无
  */
void set_motor5_enable(void)
{
    is_motor5_en = 1;
    MOTOR5_FWD_ENABLE();
    MOTOR5_REV_ENABLE();
}

/**
  * @brief  禁用电机5
  * @param  无
  * @retval 无
  */
void set_motor5_disable(void)
{
    is_motor5_en = 0;
    MOTOR5_FWD_DISABLE();
    MOTOR5_REV_DISABLE();
}

/**
  * @brief  下电机3增量式 PID 控制实现(定时调用)
  * @param  无
  * @retval 无
  */
void motor3_pid_control(void)
{
    if (is_motor3_en == 1)    			 																										 										// 电机在使能状态下才进行控制处理
    {
        float cont_val = 0;    //存储PID计算的控制值
        int temp_val = 0;          //存储处理后的控制值   																										 										// 当前控制值

        cont_val = PID_realize(&pid3, positiondown_adc_mean, Pflag3, Iflag3, Iflagz3);    																 	 						// 将Pid3的值传递给函数进行 PID 计算

        if (cont_val > 0)   	 																														 										// 判断电机方向
        {
            set_motor3_direction(MOTOR_FWD);
        }
        else
        {
            set_motor3_direction(MOTOR_REV);
        }
        temp_val = (fabs(cont_val) > PWM_MAX_PERIOD_COUNT*0.9) ? PWM_MAX_PERIOD_COUNT*0.9 : fabs(cont_val);    // 速度上限处理    判断绝对值是否大于5500的80%，大于则是5500的80%，小于则是绝对值
        set_motor3_speed(temp_val);                                                                     			 // 设置 PWM 占空比
    }
}

/**
  * @brief  上电机4增量式 PID 控制实现(定时调用)
  * @param  无
  * @retval 无
  */
void motor4_pid_control(void)
{
    if (is_motor4_en == 1)    			 																										 										// 电机在使能状态下才进行控制处理
    {
        float cont_val = 0;
        int temp_val = 0;             																										 										// 当前控制值
        if(positionup_adc_mean < 659)
        {
            positionup_adc_mean = 659;
        }
        if(positionup_adc_mean > 2124)
        {
            positionup_adc_mean = 2124;
        }
        cont_val = PID_realize(&pid4, positionup_adc_mean, Pflag4, Iflag4, Iflagz4);    																 	 							// 进行 PID 计算

        int32_t err = pid4.err;
        int32_t err_last = pid4.err_last;
        int32_t err_next = pid4.err_next;

        if (cont_val > 0)   	 																														 										// 判断电机方向
        {
            set_motor4_direction(MOTOR_REV);
        }
        else
        {
            set_motor4_direction(MOTOR_FWD);
        }
        temp_val = (fabs(cont_val) > PWM_MAX_PERIOD_COUNT*0.9) ? PWM_MAX_PERIOD_COUNT*0.9 : fabs(cont_val);    // 速度上限处理
        set_motor4_speed(temp_val);                                                                     			 // 设置 PWM 占空比
    }
}

void BLE_motor_control(void)
{
    char* redata;
    uint16_t len;
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
            if (first_char == '2')       //向下
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
                } else {
                    set_motor1_disable();
                    set_motor2_disable();
                }
            }
            if (first_char == 'b')       //设置M1、M2不同占空比
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
            if (first_char == 'c')       //设置M1、M2不同占空比
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
                } else {
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
                        LED1_TOGGLE
                        A1_control();
                    }
                    if(strcmp(second_char, "A2") == 0)
                    {
                        LED2_TOGGLE
                        A2_control();
                    }
                    if(strcmp(second_char, "A3") == 0)
                    {
                        LED3_TOGGLE
                        A3_control();
                    }
                    if(strcmp(second_char, "A4") == 0)
                    {
                        LED4_TOGGLE
                        A4_control();
                    }
                    if(strcmp(second_char, "A5") == 0)
                    {
                        LED5_TOGGLE
                        A5_control();
                    }
                    if(strcmp(second_char, "B1") == 0)
                    {
                        LED1_TOGGLE
                        B1_control();
                    }
                    if(strcmp(second_char, "B2") == 0)
                    {
                        LED2_TOGGLE
                        B2_control();
                    }
                    if(strcmp(second_char, "B3") == 0)
                    {
                        LED3_TOGGLE
                        B3_control();
                    }
                    if(strcmp(second_char, "B4") == 0)
                    {
                        LED4_TOGGLE
                        B4_control();
                    }
                    if(strcmp(second_char, "B5") == 0)
                    {
                        LED5_TOGGLE
                        B5_control();
                    }
                    if(strcmp(second_char, "C1") == 0)
                    {
                        LED1_TOGGLE
                        C1_control();
                    }
                    if(strcmp(second_char, "C2") == 0)
                    {
                        LED2_TOGGLE
                        C2_control();
                    }
                    if(strcmp(second_char, "C3") == 0)
                    {
                        LED3_TOGGLE
                        C3_control();
                    }
                    if(strcmp(second_char, "C4") == 0)
                    {
                        LED4_TOGGLE
                        C4_control();
                    }
                    if(strcmp(second_char, "C5") == 0)
                    {
                        LED5_TOGGLE
                        C5_control();
                    }
                    if(strcmp(second_char, "D1") == 0)
                    {
                        LED1_TOGGLE
                        D1_control();
                    }
                    if(strcmp(second_char, "D2") == 0)
                    {
                        LED2_TOGGLE
                        D2_control();
                    }
                    if(strcmp(second_char, "D3") == 0)
                    {
                        LED3_TOGGLE
                        D3_control();
                    }
                    if(strcmp(second_char, "D4") == 0)
                    {
                        LED4_TOGGLE
                        D4_control();
                    }
                    if(strcmp(second_char, "D5") == 0)
                    {
                        LED5_TOGGLE
                        D5_control();
                    }
                    if(strcmp(second_char, "E1") == 0)
                    {
                        LED1_TOGGLE
                        E1_control();
                    }
                    if(strcmp(second_char, "E2") == 0)
                    {
                        LED2_TOGGLE
                        E2_control();
                    }
                    if(strcmp(second_char, "E3") == 0)
                    {
                        LED3_TOGGLE
                        E3_control();
                    }
                    if(strcmp(second_char, "E4") == 0)
                    {
                        LED4_TOGGLE
                        E4_control();
                    }
                    if(strcmp(second_char, "E5") == 0)
                    {
                        LED5_TOGGLE
                        E5_control();
                    }
                    break;
//                    case '2':
//                        clean_rebuff();
//                        if(!randomflag)
//                        {
//                            LED5_ON
//                            randomflag = 1;
//                        }
//                        else
//                        {
//                            LED5_OFF
//                            LED3_OFF
//                            randomflag = 0;
//                        }
//                        break;
            }
            /*处理数据后，清空接收蓝牙模块数据的缓冲区*/
            clean_rebuff();
            Task_Delay[0] = 100;                            //此值每1ms会减1，减到0才可以重新进来这里，所以执行的周期是100ms
        }
        BLE_WAKEUP_HIGHT;
    }
}

void upward_control(void)
{
    LED5_TOGGLE
    uint16_t temp_val = get_pid_target(&pid4); // 获取当前PID目标值
    if (temp_val >= 700) {
        temp_val -= 20; // 每次减少150
        if (temp_val < 700) {
            temp_val = 700; // 如果减少后的值小于700，则将其设置为700
        }
        set_pid_target4(&pid4, temp_val); // 设置新的PID目标值
    }
}

void downward_control(void)
{
    LED2_TOGGLE
    uint16_t temp_val = get_pid_target(&pid4); // 获取当前PID目标值
    if (temp_val <= 2100)
    {
        temp_val += 20; // 每次增加150
        if (temp_val > 2100)
        {
            temp_val = 2100; // 如果增加后的值大于2100，则将其设置为2100
        }
        set_pid_target4(&pid4, temp_val); // 设置新的PID目标值
    }
}

void left_control(void)
{
    LED3_TOGGLE
    uint16_t temp_val = get_pid_target(&pid3); // 获取当前PID目标值
    if(temp_val >= 700)
    {
        temp_val -= 20; // 每次减少150
        if(temp_val < 700)
        {
            temp_val = 700; // 如果减少后的值小于700，则将其设置为700
        }
        set_pid_target3(&pid3, temp_val); // 设置新的PID目标值
    }
}

void right_control(void)
{
    LED4_TOGGLE
    uint16_t temp_val = get_pid_target(&pid3); // 获取当前PID目标值
    if(temp_val <= 2200)
    {
        temp_val += 20; // 每次增加100
        if(temp_val > 2200) {
            temp_val = 2200; // 如果增加后的值大于2100，则将其设置为2100
        }
        set_pid_target3(&pid3, temp_val); // 设置新的PID目标值
    }
}

void motor1_motor2_control(void)
{
    LED5_TOGGLE
    if(!is_motor1_en && !is_motor2_en)
    {
        set_motor1_enable();
        set_motor1_direction(MOTOR_FWD);
        set_motor1_speed(1800);

        set_motor2_enable();
        set_motor2_direction(MOTOR_REV);
        set_motor2_speed(1800);
    }
    else
    {
        set_motor1_disable();
        set_motor2_disable();
    }
}

void motor1_motor2_speedup(void)
{
    __IO uint16_t motor1_ChannelPulse = 2000; // 9000
    __IO uint16_t motor2_ChannelPulse = 2000;

    LED5_TOGGLE
    motor1_ChannelPulse += 100;
    motor2_ChannelPulse += 100;
    if((motor1_ChannelPulse > PWM_MAX_PERIOD_COUNT)||(motor2_ChannelPulse > PWM_MAX_PERIOD_COUNT))
    {
        motor1_ChannelPulse = PWM_MAX_PERIOD_COUNT;
        motor2_ChannelPulse = PWM_MAX_PERIOD_COUNT;
    }
    set_motor1_speed(motor1_ChannelPulse);
    set_motor2_speed(motor2_ChannelPulse);
}

void motor1_motor2_slowdown(void)
{
    __IO uint16_t motor1_ChannelPulse = 2000; // 9000
    __IO uint16_t motor2_ChannelPulse = 2000;
    LED5_TOGGLE
    motor1_ChannelPulse -= 100;
    motor2_ChannelPulse -= 100;
    if((motor1_ChannelPulse < PWM_MAX_PERIOD_COUNT/10)||(motor2_ChannelPulse < PWM_MAX_PERIOD_COUNT/10))
    {
        motor1_ChannelPulse = 0;
        motor2_ChannelPulse = 0;
    }

    set_motor1_speed(motor1_ChannelPulse);
    set_motor2_speed(motor2_ChannelPulse);
}

void motor_reset(void)
{
    LED5_TOGGLE
    __set_FAULTMASK(1);
    NVIC_SystemReset();
}

void motor5_control(void)
{
    LED5_TOGGLE
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

void A1_control(void)
{
    set_motor1_enable();                            //A1
    set_motor1_direction(MOTOR_FWD);
    set_motor1_speed(1900);
    set_motor2_enable();
    set_motor2_direction(MOTOR_REV);
    set_motor2_speed(1900);

    set_pid_target3(&pid3, 2500);
}

void A2_control(void)
{
    set_motor1_enable();                            //A2
    set_motor1_direction(MOTOR_FWD);
    set_motor1_speed(2100);
    set_motor2_enable();
    set_motor2_direction(MOTOR_REV);
    set_motor2_speed(2100);

    set_pid_target3(&pid3, 1900);
}

void A3_control(void)
{
    set_motor1_enable();                            //A3
    set_motor1_direction(MOTOR_FWD);
    set_motor1_speed(1800);
    set_motor2_enable();
    set_motor2_direction(MOTOR_REV);
    set_motor2_speed(1800);

    set_pid_target3(&pid3, 1738);
}

void A4_control(void)
{
    set_motor1_enable();                            //A4
    set_motor1_direction(MOTOR_FWD);
    set_motor1_speed(1800);
    set_motor2_enable();
    set_motor2_direction(MOTOR_REV);
    set_motor2_speed(1800);

    set_pid_target3(&pid3, 1290);
}

void A5_control(void)
{
    set_motor1_enable();                            //A5
    set_motor1_direction(MOTOR_FWD);
    set_motor1_speed(1800);
    set_motor2_enable();
    set_motor2_direction(MOTOR_REV);
    set_motor2_speed(1800);

    set_pid_target3(&pid3, 745);
}

void B1_control(void)
{
    set_motor1_enable();
    set_motor1_direction(MOTOR_FWD);
    set_motor1_speed(2100);                         //B1
    set_motor2_enable();
    set_motor2_direction(MOTOR_REV);
    set_motor2_speed(2100);

    set_pid_target3(&pid3, 2250);

}

void B2_control(void)
{
    set_motor1_enable();
    set_motor1_direction(MOTOR_FWD);
    set_motor1_speed(2100);                         //B2
    set_motor2_enable();
    set_motor2_direction(MOTOR_REV);
    set_motor2_speed(2100);

    set_pid_target3(&pid3, 1900);
}

void B3_control(void)
{
    set_motor1_enable();
    set_motor1_direction(MOTOR_FWD);
    set_motor1_speed(2100);                         //B3
    set_motor2_enable();
    set_motor2_direction(MOTOR_REV);
    set_motor2_speed(2100);

    set_pid_target3(&pid3, 1591);
}

void B4_control(void)
{
    set_motor1_enable();
    set_motor1_direction(MOTOR_FWD);
    set_motor1_speed(2200);                         //B4
    set_motor2_enable();
    set_motor2_direction(MOTOR_REV);
    set_motor2_speed(2200);

    set_pid_target3(&pid3, 745);

}

void B5_control(void)
{
    set_motor1_enable();
    set_motor1_direction(MOTOR_FWD);
    set_motor1_speed(2100);                         //B5
    set_motor2_enable();
    set_motor2_direction(MOTOR_REV);
    set_motor2_speed(2100);

    set_pid_target3(&pid3, 745);

}

void C1_control(void)
{
    set_motor1_enable();
    set_motor1_direction(MOTOR_FWD);
    set_motor1_speed(2400);                         //C1
    set_motor2_enable();
    set_motor2_direction(MOTOR_REV);
    set_motor2_speed(2400);

    set_pid_target3(&pid3, 2000);
}

void C2_control(void)
{
    set_motor1_enable();
    set_motor1_direction(MOTOR_FWD);
    set_motor1_speed(2400);                         //C2
    set_motor2_enable();
    set_motor2_direction(MOTOR_REV);
    set_motor2_speed(2400);

    set_pid_target3(&pid3, 1750);
}

void C3_control(void)
{
    set_motor1_enable();
    set_motor1_direction(MOTOR_FWD);
    set_motor1_speed(2400);                         //C3
    set_motor2_enable();
    set_motor2_direction(MOTOR_REV);
    set_motor2_speed(2400);
    HAL_Delay(1500);

    set_pid_target3(&pid3, 1580);
}

void C4_control(void)
{
    set_motor1_enable();
    set_motor1_direction(MOTOR_FWD);
    set_motor1_speed(2400);                         //C4
    set_motor2_enable();
    set_motor2_direction(MOTOR_REV);
    set_motor2_speed(2400);

    set_pid_target3(&pid3, 1200);
}

void C5_control(void)
{
    set_motor1_enable();
    set_motor1_direction(MOTOR_FWD);
    set_motor1_speed(2400);                         //C5
    set_motor2_enable();
    set_motor2_direction(MOTOR_REV);
    set_motor2_speed(2400);


    set_pid_target3(&pid3, 1100);
}

void D1_control(void)
{
    set_motor1_enable();
    set_motor1_direction(MOTOR_FWD);
    set_motor1_speed(2400);                         //D1
    set_motor2_enable();
    set_motor2_direction(MOTOR_REV);
    set_motor2_speed(2400);

    set_pid_target3(&pid3, 2000);
}

void D2_control(void)
{
    set_motor1_enable();
    set_motor1_direction(MOTOR_FWD);
    set_motor1_speed(2400);                         //D2
    set_motor2_enable();
    set_motor2_direction(MOTOR_REV);
    set_motor2_speed(2400);

    set_pid_target3(&pid3, 1780);
}

void D3_control(void)
{
    set_motor1_enable();
    set_motor1_direction(MOTOR_FWD);
    set_motor1_speed(2400);                         //D3
    set_motor2_enable();
    set_motor2_direction(MOTOR_REV);
    set_motor2_speed(2400);

    set_pid_target3(&pid3, 1530);
}

void D4_control(void)
{
    set_motor1_enable();
    set_motor1_direction(MOTOR_FWD);
    set_motor1_speed(2400);                         //D4
    set_motor2_enable();
    set_motor2_direction(MOTOR_REV);
    set_motor2_speed(2400);

    set_pid_target3(&pid3, 1300);
}

void D5_control(void)
{
    set_motor1_enable();
    set_motor1_direction(MOTOR_FWD);
    set_motor1_speed(2400);                         //D5
    set_motor2_enable();
    set_motor2_direction(MOTOR_REV);
    set_motor2_speed(2400);

    set_pid_target3(&pid3, 1100);
}

void E1_control(void)
{
    set_motor1_enable();
    set_motor1_direction(MOTOR_FWD);
    set_motor1_speed(2400);                         //E1
    set_motor2_enable();
    set_motor2_direction(MOTOR_REV);
    set_motor2_speed(2400);

    set_pid_target3(&pid3, 1920);
}

void E2_control(void)
{
    set_motor1_enable();
    set_motor1_direction(MOTOR_FWD);
    set_motor1_speed(2400);                         //E2
    set_motor2_enable();
    set_motor2_direction(MOTOR_REV);
    set_motor2_speed(2400);

    set_pid_target3(&pid3, 1720);
}

void E3_control(void)
{
    set_motor1_enable();
    set_motor1_direction(MOTOR_FWD);
    set_motor1_speed(2400);                         //E3
    set_motor2_enable();
    set_motor2_direction(MOTOR_REV);
    set_motor2_speed(2400);

    set_pid_target3(&pid3, 1560);
}

void E4_control(void)
{
    set_motor1_enable();
    set_motor1_direction(MOTOR_FWD);
    set_motor1_speed(2400);                         //E4
    set_motor2_enable();
    set_motor2_direction(MOTOR_REV);
    set_motor2_speed(2400);

    set_pid_target3(&pid3, 1230);
}

void E5_control(void)
{
    set_motor1_enable();
    set_motor1_direction(MOTOR_FWD);
    set_motor1_speed(2400);                         //E5
    set_motor2_enable();
    set_motor2_direction(MOTOR_REV);
    set_motor2_speed(2400);

    set_pid_target3(&pid3, 800);
}

