#include "bsp_adc.h"
#include "bsp_usart.h"

ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;
DMA_HandleTypeDef DMA_Init_Handle;
static uint16_t adc_buff[ADC_NUM_MAX];     			    /** 电压采集缓冲区 */
int32_t positionup_adc_mean = 0;   					    /** 位置上电压 ADC 采样结果平均值 */
int32_t positiondown_adc_mean = 0; 					    /** 位置下电压 ADC 采样结果平均值 */
int32_t Dropping_adc_mean = 0; 					        /** 落球电机电压 ADC 采样结果平均值 */
int32_t Rotation1_adc_mean = 0;
__IO uint32_t Dropping_ADC_ConvertedValue;
__IO uint32_t Rotation1_ADC_ConvertedValue;

/**
  * @brief  ADC 通道引脚初始化
  * @param  无
  * @retval 无
  */
static void ADC_GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    POSITONUP_ADC_GPIO_CLK_ENABLE();
    POSITONDOWN_ADC_GPIO_CLK_ENABLE();

    GPIO_InitStructure.Pin = POSITONUP_ADC_GPIO_PIN;
    GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStructure.Pull = GPIO_NOPULL ;             /** 不上拉不下拉 */
    HAL_GPIO_Init(POSITONUP_ADC_GPIO_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = POSITONDOWN_ADC_GPIO_PIN;
    HAL_GPIO_Init(POSITONDOWN_ADC_GPIO_PORT, &GPIO_InitStructure);

}

/**
  * @brief  ADC 通道引脚初始化
  * @param  无
  * @retval 无
  */
static void Dropping_ADC_GPIO_Config(void)
{
    GPIO_InitTypeDef Dropping_GPIO_InitStructure;

    Dropping_ADC_GPIO_CLK_ENABLE();

    Dropping_GPIO_InitStructure.Pin = Dropping_ADC_GPIO_PIN;
    Dropping_GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
    Dropping_GPIO_InitStructure.Pull = GPIO_NOPULL ;             /** 不上拉不下拉 */
    HAL_GPIO_Init(Dropping_ADC_GPIO_PORT, &Dropping_GPIO_InitStructure);
}

/**
  * @brief  ADC 通道引脚初始化
  * @param  无
  * @retval 无
  */
static void Rotation1_ADC_GPIO_Config(void)
{
    GPIO_InitTypeDef Rotation1_GPIO_InitStructure;
    Rotation1_ADC_GPIO_CLK_ENABLE();

    Rotation1_GPIO_InitStructure.Pin = Rotation1_ADC_GPIO_PIN;
    Rotation1_GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
    Rotation1_GPIO_InitStructure.Pull = GPIO_NOPULL ;             /** 不上拉不下拉 */
    HAL_GPIO_Init(Rotation1_ADC_GPIO_PORT, &Rotation1_GPIO_InitStructure);
}

void ADC_DMA_INIT(void)
{
    /** ------------------DMA Init 结构体参数 初始化-------------------------- */
    // 开启DMA时钟
    POSITION_ADC_DMA_CLK_ENABLE();
    // 数据传输通道
    DMA_Init_Handle.Instance = POSITION_ADC_DMA_STREAM;
    // 数据传输方向为外设到存储器
    DMA_Init_Handle.Init.Direction = DMA_PERIPH_TO_MEMORY;
    // 外设寄存器只有一个，地址不用递增
    DMA_Init_Handle.Init.PeriphInc = DMA_PINC_DISABLE;
    // 存储器地址固定
    DMA_Init_Handle.Init.MemInc = DMA_MINC_ENABLE;
    // 外设数据大小为半字，即两个字节
    DMA_Init_Handle.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    //	存储器数据大小也为半字，跟外设数据大小相同
    DMA_Init_Handle.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    // 循环传输模式
    DMA_Init_Handle.Init.Mode = DMA_CIRCULAR;
    // DMA 传输通道优先级为高，当使用一个DMA通道时，优先级设置不影响
    DMA_Init_Handle.Init.Priority = DMA_PRIORITY_HIGH;

    HAL_DMA_Init(&DMA_Init_Handle);

    __HAL_LINKDMA(&hadc1,DMA_Handle,DMA_Init_Handle);
}

/**
  * @brief  ADC 和 DMA 初始化
  * @param  无
  * @retval 无
  */
static void ADC_Mode_Config(void)
{
    // 开启ADC时钟
    POSITION_ADC_CLK_ENABLE();
    // ADC1
    hadc1.Instance = POSITION_ADC;
    // 使能扫描模式，多通道采集才需要
    hadc1.Init.ScanConvMode = ENABLE;
    // 连续转换
    hadc1.Init.ContinuousConvMode = ENABLE;
    // 非连续转换
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    // 非连续转换个数
    hadc1.Init.NbrOfDiscConversion   = 0;
    //使用软件触发
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    //数据右对齐
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    //转换通道 2个
    hadc1.Init.NbrOfConversion = 2;
    // 初始化ADC
    HAL_ADC_Init(&hadc1);

    ADC_ChannelConfTypeDef ADC_Config;

    ADC_Config.Channel      = POSITONUP_ADC_CHANNEL;
    ADC_Config.Rank         = 1;    // 配置 ADC 通道转换顺序为1，第一个转换，采样时间为3个时钟周期
    ADC_Config.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;    // 采样时间间隔

    HAL_ADC_ConfigChannel(&hadc1, &ADC_Config);

    ADC_Config.Channel 			= POSITONDOWN_ADC_CHANNEL;
    ADC_Config.Rank 			= 2;
    ADC_Config.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;    // 采样时间间隔

    if (HAL_ADC_ConfigChannel(&hadc1, &ADC_Config) != HAL_OK)
    {
        while(1);
    }
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&adc_buff, ADC_NUM_MAX);
}

static void Dropping_ADC_Mode_Config(void)
{
    // 开启ADC时钟
    Dropping_ADC_CLK_ENABLE();
    // ADC1
    hadc2.Instance = Dropping_ADC;
    // 使能扫描模式，多通道采集才需要
    hadc2.Init.ScanConvMode = DISABLE;
    // 连续转换
    hadc2.Init.ContinuousConvMode = ENABLE;
    // 非连续转换
    hadc2.Init.DiscontinuousConvMode = DISABLE;
    // 非连续转换个数
    hadc2.Init.NbrOfDiscConversion   = 0;
    //使用软件触发
    hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    //数据右对齐
    hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    //转换通道 0个
    hadc2.Init.NbrOfConversion = 0;
    // 初始化ADC
    HAL_ADC_Init(&hadc2);

    ADC_ChannelConfTypeDef Dropping_ADC_Config;

    Dropping_ADC_Config.Channel      = Dropping_ADC_CHANNEL;
    Dropping_ADC_Config.Rank         = 1;    // 配置 ADC 通道转换顺序为1，第一个转换，采样时间为3个时钟周期
    Dropping_ADC_Config.SamplingTime = ADC_SAMPLETIME_239CYCLES_5  ;    // 采样时间间隔

    HAL_ADC_ConfigChannel(&hadc2, &Dropping_ADC_Config);

    HAL_ADC_Start_IT(&hadc2);
}

static void Rotation1_ADC_Mode_Config(void)
{
    // 开启ADC时钟
    Rotation_ADC_CLK_ENABLE();
    // ADC1
    hadc3.Instance = Rotation_ADC;
    // 使能扫描模式，多通道采集才需要
    hadc3.Init.ScanConvMode = DISABLE;
    // 连续转换
    hadc3.Init.ContinuousConvMode = ENABLE;
    // 非连续转换
    hadc3.Init.DiscontinuousConvMode = DISABLE;
    // 非连续转换个数
    hadc3.Init.NbrOfDiscConversion   = 0;
    //使用软件触发
    hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    //数据右对齐
    hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    //转换通道 0个
    hadc3.Init.NbrOfConversion = 0;
    // 初始化ADC
    HAL_ADC_Init(&hadc3);

    ADC_ChannelConfTypeDef Rotation_ADC_Config;

    Rotation_ADC_Config.Channel      = Rotation1_ADC_CHANNEL;
    Rotation_ADC_Config.Rank         = 1;    // 配置 ADC 通道转换顺序为5，第一个转换，采样时间为3个时钟周期
    Rotation_ADC_Config.SamplingTime = ADC_SAMPLETIME_71CYCLES_5  ;    // 采样时间间隔

    HAL_ADC_ConfigChannel(&hadc3, &Rotation_ADC_Config);

    HAL_ADC_Start_IT(&hadc3);
}

void ADC1_Init(void)
{
    ADC_GPIO_Config();
    ADC_DMA_INIT();
    ADC_Mode_Config();
}

void ADC2_Init(void)
{
    Dropping_ADC_GPIO_Config();
    Dropping_ADC_Mode_Config();
}

void Rotation_ADC_Init(void)
{
    Rotation1_ADC_GPIO_Config();
    Rotation1_ADC_Mode_Config();
}

/**
  * @brief  常规转换在非阻塞模式下完成回调
  * @param  hadc: ADC  句柄.
  * @retval 无
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if(hadc == &hadc1)
    {
        uint32_t adc_mean = 0;        //用于储存ADC的平均值

        HAL_ADC_Stop_DMA(hadc);       // 停止 ADC 采样，处理完一次数据在继续采样

        for(uint32_t count = 0; count < ADC_NUM_MAX; count += 2)            //开始一个循环，从0开始，每次增加2，直到 ADC_NUM_MAX。这个循环用于计算ADC转换结果的平均值。
        {
            adc_mean += (uint32_t)adc_buff[count];                          //在每次迭代中，将 adc_buff 数组中的元素累加到 adc_mean 变量中。
        }

        positionup_adc_mean = adc_mean / (ADC_NUM_MAX / 2);                 //计算ADC转换结果的平均值。这个平均值被存储到 positionup_adc_mean 变量中。

        adc_mean = 0;

        for(uint32_t count = 1; count < ADC_NUM_MAX; count += 2)
        {
            adc_mean += (uint32_t)adc_buff[count];
        }

        positiondown_adc_mean = adc_mean / (ADC_NUM_MAX / 2);    // 计算ADC转换结果的平均值。这个平均值被存储到 positiondown_adc_mean 变量中。

        HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&adc_buff, ADC_NUM_MAX);    // 开始 ADC 采样
    }
    if(hadc == &hadc2)
    {
        // 在这里处理ADC转换完成中断，读取ADC值等操作
        Dropping_ADC_ConvertedValue = HAL_ADC_GetValue(hadc);
        // 处理读取到的ADC值
        Dropping_adc_mean = Dropping_ADC_ConvertedValue;
    }
    if(hadc == &hadc3)
    {
//       在这里处理ADC转换完成中断，读取ADC值等操作
        Rotation1_ADC_ConvertedValue = HAL_ADC_GetValue(hadc);
        // 处理读取到的ADC值
        Rotation1_adc_mean = Rotation1_ADC_ConvertedValue;

    }
}

/**
  * @brief  获取电压值
  * @param  无
  * @retval 转换电压值
  */
float get_positionup_val(void)
{
    float vdc = GET_ADC_VAL(positionup_adc_mean);      // 获取电压值
    return vdc;
}

/**
  * @brief  获取电压值
  * @param  无
  * @retval 转换电压值
  */
float get_positiondown_val(void)
{
    float vdc = GET_ADC_VAL(positiondown_adc_mean);      // 获取电压值
    return vdc;
}

/**
  * @brief  获取电压值
  * @param  无
  * @retval 转换电压值
  */
float get_ADC_val(void)
{
    float vdc = GET_ADC_VAL(Dropping_adc_mean);      // 获取电压值
    return vdc;
}