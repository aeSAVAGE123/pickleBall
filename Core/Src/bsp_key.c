
#include "bsp_key.h"
#include "bsp_pid.h"
#include "bsp_led.h"
#include "bsp_usart.h"
/**
  * @brief  配置按键用到的I/O口
  * @param  无
  * @retval 无
  */
void Key_GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /*开启按键GPIO口的时钟*/
    KEY1_GPIO_CLK_ENABLE();
    KEY2_GPIO_CLK_ENABLE();
    KEY3_GPIO_CLK_ENABLE();
    KEY4_GPIO_CLK_ENABLE();
    KEY5_GPIO_CLK_ENABLE();

    /*选择按键的引脚*/
    GPIO_InitStructure.Pin = KEY1_PIN;

    /*设置引脚为输入模式*/
    GPIO_InitStructure.Mode = GPIO_MODE_INPUT;

    /*设置引脚不上拉也不下拉*/
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    /*使用上面的结构体初始化按键*/
    HAL_GPIO_Init(KEY1_GPIO_PORT, &GPIO_InitStructure);
    /*选择按键的引脚*/
    GPIO_InitStructure.Pin = KEY2_PIN;
    /*使用上面的结构体初始化按键*/
    HAL_GPIO_Init(KEY2_GPIO_PORT, &GPIO_InitStructure);

    /*选择按键的引脚*/
    GPIO_InitStructure.Pin = KEY3_PIN;
    /*使用上面的结构体初始化按键*/
    HAL_GPIO_Init(KEY3_GPIO_PORT, &GPIO_InitStructure);

    /*选择按键的引脚*/
    GPIO_InitStructure.Pin = KEY4_PIN;
    /*使用上面的结构体初始化按键*/
    HAL_GPIO_Init(KEY4_GPIO_PORT, &GPIO_InitStructure);

    /*选择按键的引脚*/
    GPIO_InitStructure.Pin = KEY5_PIN;
    /*使用上面的结构体初始化按键*/
    HAL_GPIO_Init(KEY5_GPIO_PORT, &GPIO_InitStructure);
}

/**
  * @brief   检测是否有按键按下
  * @param   具体的端口和端口位
  *		@arg GPIOx: x可以是（A...G）
  *		@arg GPIO_PIN 可以是GPIO_PIN_x（x可以是1...16）
  * @retval  按键的状态
  *		@arg KEY_ON:按键按下
  *		@arg KEY_OFF:按键没按下
  */
uint8_t Key_Scan(GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin)
{
    /*检测是否有按键按下 */
    if(HAL_GPIO_ReadPin(GPIOx,GPIO_Pin) == KEY_ON )
    {
        /*等待按键释放 */
        HAL_Delay(20);
        while(HAL_GPIO_ReadPin(GPIOx,GPIO_Pin) == KEY_ON);
        HAL_Delay(20);
        return 	KEY_ON;
    }
    else
        return KEY_OFF;
}

void Key_control(void)
{
    /* 扫描KEY1 */
    if( Key_Scan(KEY1_GPIO_PORT, KEY1_PIN) == KEY_ON)
    {
        set_computer_Speed_Location_value(Send_Speed_CMD, 1800);
        uint16_t temp_val = get_pid_target(&pid4); // 获取当前PID目标值
        if (temp_val >= 700) {
            temp_val -= 50; // 每次减少50
            if (temp_val < 700) {
                temp_val = 700; // 如果减少后的值小于700，则将其设置为700
            }
            set_pid_target4(&pid4, temp_val); // 设置新的PID目标值
        }
        LED5_TOGGLE
    }

    /* 扫描KEY2 */
    if( Key_Scan(KEY2_GPIO_PORT, KEY2_PIN) == KEY_ON)
    {
        uint16_t temp_val = get_pid_target(&pid4); // 获取当前PID目标值
        if (temp_val <= 2100) {
            temp_val += 50; // 每次增加50
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
            temp_val -= 50; // 每次减少50
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
            temp_val += 50; // 每次增加50
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
        LED5_TOGGLE
    }
}
/*********************************************END OF FILE**********************/

