#include "protocol.h"
#include "bsp_basic_tim.h"


TIM_HandleTypeDef TIM_TimeBaseStructure1;
/**
 * @brief  基本定时器 TIMx,x[6,7]中断优先级配置
 * @param  无
 * @retval 无
 */
static void TIMx_NVIC_Configuration(void)
{
    //设置抢占优先级，子优先级
    HAL_NVIC_SetPriority(BASIC_TIM_IRQn, 0, 3);
    // 设置中断来源
    HAL_NVIC_EnableIRQ(BASIC_TIM_IRQn);
}

/*
 * 注意：TIM_TimeBaseInitTypeDef结构体里面有5个成员，TIM6和TIM7的寄存器里面只有
 * TIM_Prescaler和TIM_Period，所以使用TIM6和TIM7的时候只需初始化这两个成员即可，
 * 另外三个成员是通用定时器和高级定时器才有.
 *-----------------------------------------------------------------------------
 * TIM_Prescaler         都有
 * TIM_CounterMode			 TIMx,x[6,7]没有，其他都有（基本定时器）
 * TIM_Period            都有
 * TIM_ClockDivision     TIMx,x[6,7]没有，其他都有(基本定时器)
 * TIM_RepetitionCounter TIMx,x[1,8]才有(高级定时器)
 *-----------------------------------------------------------------------------
 */
static void TIM_Mode_Config(void)
{
    // 开启TIMx_CLK,x[6,7]
    BASIC_TIM_CLK_ENABLE();

    TIM_TimeBaseStructure1.Instance = BASIC_TIM;
    /* 累计 TIM_Period个后产生一个更新或者中断*/
    //当定时器从0计数到4999，即为5000次，为一个定时周期
    TIM_TimeBaseStructure1.Init.Period = BASIC_PERIOD_COUNT - 1;            // 5000
    //定时器时钟源TIMxCLK = 2 * PCLK1
    //				PCLK1 = HCLK / 2
    //				=> TIMxCLK=HCLK/2=SystemCoreClock/2*2=72MHz
    // 设定定时器频率为=TIMxCLK/(TIM_Prescaler+1)=100KHz
    TIM_TimeBaseStructure1.Init.Prescaler = BASIC_PRESCALER_COUNT - 1;      // 720
    TIM_TimeBaseStructure1.Init.CounterMode = TIM_COUNTERMODE_UP;           // 向上计数
    TIM_TimeBaseStructure1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;     // 时钟分频

    // 初始化定时器TIMx, x[2,3,4,5]
    HAL_TIM_Base_Init(&TIM_TimeBaseStructure1);

    // 开启定时器更新中断
    HAL_TIM_Base_Start_IT(&TIM_TimeBaseStructure1);
}

/**
  * @brief  初始化基本定时器定时，1ms产生一次中断
  * @param  无
  * @retval 无
  */
void Basic_TIMx_Configuration(void)
{
    TIMx_NVIC_Configuration();
    TIM_Mode_Config();
//    uint32_t temp = GET_BASIC_TIM_PERIOD();     // 计算周期，单位ms
//    set_computer_value(SEND_PERIOD_CMD, CURVES_CH1, &temp, 1);     // 给通道 1 发送目标值
}

/*********************************************END OF FILE**********************/
