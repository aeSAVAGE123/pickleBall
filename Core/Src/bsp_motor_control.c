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



