/**
 ****************************************************************************************************
 * @file        adc.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.1
 * @date        2022-09-06
 * @brief       ADC 驱动代码
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 * @attention
 *
 * 实验平台:正点原子 阿波罗 H743开发板
 * 在线视频:www.yuanzige.com
 * 技术论坛:www.openedv.com
 * 公司网址:www.alientek.com
 * 购买地址:openedv.taobao.com
 *
 * 修改说明
 * V1.0 20220906
 * 第一次发布
 * V1.1 20220906
 * 1,支持ADC单通道DMA采集 
 * 2,新增adc_dma_init和adc_dma_enable函数
 *
 ****************************************************************************************************
 */

#include "./BSP/ADC/adc.h"
#include "./SYSTEM/delay/delay.h"
#include <string.h>


ADC_HandleTypeDef g_adc_handle;           /* ADC句柄 */

/**
 * @brief       ADC初始化函数
 * @param       无
 * @retval      无
 */
void adc_init(void)
{ 
    g_adc_handle.Instance = ADC_ADCX;
    g_adc_handle.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV2;                /* 分频，ADCCLK=PER_CK/2=64/2=32MHZ */
    g_adc_handle.Init.Resolution = ADC_RESOLUTION_16B;                      /* 16位模式 */
    g_adc_handle.Init.ScanConvMode = DISABLE;                               /* 非扫描模式 */
    g_adc_handle.Init.EOCSelection = ADC_EOC_SINGLE_CONV;                   /* 关闭EOC中断 */
    g_adc_handle.Init.LowPowerAutoWait = DISABLE;                           /* 自动低功耗关闭 */
    g_adc_handle.Init.ContinuousConvMode = DISABLE;                         /* 关闭连续转换 */
    g_adc_handle.Init.NbrOfConversion = 1;                                  /* 1个转换在规则序列中 也就是只转换规则序列1 */
    g_adc_handle.Init.DiscontinuousConvMode = DISABLE;                      /* 禁止不连续采样模式 */
    g_adc_handle.Init.NbrOfDiscConversion = 0;                              /* 不连续采样通道数为0 */
    g_adc_handle.Init.ExternalTrigConv = ADC_SOFTWARE_START;                /* 软件触发 */
    g_adc_handle.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE; /* 使用软件触发 */
    g_adc_handle.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;                   /* 有新的数据时直接覆盖掉旧数据 */
    g_adc_handle.Init.OversamplingMode = DISABLE;                           /* 过采样关闭 */
    g_adc_handle.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;     /* 规则通道的数据仅仅保存在DR寄存器里面 */
    HAL_ADC_Init(&g_adc_handle);                                            /* 初始化  */

    HAL_ADCEx_Calibration_Start(&g_adc_handle, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED); /* ADC校准 */
}

/**
 * @brief       ADC底层驱动，引脚配置，时钟使能
 * @note        此函数会被HAL_ADC_Init()调用
 * @param       hadc:ADC句柄
 * @retval      无
 */
void HAL_ADC_MspInit(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC_ADCX)
    {
        GPIO_InitTypeDef gpio_init_struct;

        ADC_ADCX_CHY_CLK_ENABLE();                          /* 使能ADC1/2时钟 */
        ADC_ADCX_CHY_GPIO_CLK_ENABLE();                     /* 开启GPIOA时钟 */

        __HAL_RCC_ADC_CONFIG(RCC_ADCCLKSOURCE_CLKP);        /* ADC外设时钟选择 */
        
        gpio_init_struct.Pin = ADC_ADCX_CHY_GPIO_PIN;       /* PA5 */
        gpio_init_struct.Mode = GPIO_MODE_ANALOG;           /* 模拟 */
        gpio_init_struct.Pull = GPIO_NOPULL;                /* 不带上下拉 */
        HAL_GPIO_Init(ADC_ADCX_CHY_GPIO_PORT, &gpio_init_struct);
    }
}

/**
 * @brief       获得ADC转换后的结果
 * @param       ch: 通道值 0~19，取值范围为：ADC_CHANNEL_0~ADC_CHANNEL_19
 * @retval      转换结果
 */
uint32_t adc_get_result(uint32_t ch)
{
    ADC_ChannelConfTypeDef adc_ch_conf = {0};

    adc_ch_conf.Channel = ch;                               /* 通道 */
    adc_ch_conf.Rank = ADC_REGULAR_RANK_1;                  /* 序列 */
    adc_ch_conf.SamplingTime = ADC_SAMPLETIME_810CYCLES_5;  /* 采样时间，设置最大采样周期: 810.5个ADC周期 */
    adc_ch_conf.SingleDiff = ADC_SINGLE_ENDED;              /* 单边采集 */
    adc_ch_conf.OffsetNumber = ADC_OFFSET_NONE;             /* 不使用偏移量的通道 */
    adc_ch_conf.Offset = 0;                                 /* 偏移量为0 */
    HAL_ADC_ConfigChannel(&g_adc_handle, &adc_ch_conf);     /* 通道配置 */

    HAL_ADC_Start(&g_adc_handle);                           /* 开启ADC */
    HAL_ADC_PollForConversion(&g_adc_handle, 10);           /* 轮询转换 */
    return HAL_ADC_GetValue(&g_adc_handle);                 /* 返回最近一次ADC1规则组的转换结果 */
}

/**
 * @brief       获取指定通道的转换值，取times次,然后平均
 * @param       ch: 通道值 0~19
 * @param       times:获取次数
 * @retval      返回值:通道ch的times次转换结果平均值
 */
uint32_t adc_get_result_average(uint32_t ch, uint8_t times)
{
    uint32_t temp_val = 0;
    uint8_t t;

    for (t = 0; t < times; t++)    /* 获取times次数据 */
    {
        temp_val += adc_get_result(ch);
        delay_ms(5);
    }

    return temp_val / times;       /* 返回平均值 */
} 

/*************************************** 以下是单通道ADC采集(DMA读取)程序 *****************************************/

ADC_HandleTypeDef g_adc_dma_handle;     /* 与DMA关联的ADC句柄 */
DMA_HandleTypeDef g_dma_adc_handle;     /* 与ADC关联的DMA句柄 */
static TIM_HandleTypeDef g_adc_tim_handle;
static uint16_t g_adc_dma_snapshot[ADC_DMA_MAX_SAMPLES] __attribute__((section(".dma_buffer"), aligned(32)));
static uint16_t *g_adc_dma_mem = NULL;
static uint16_t g_adc_dma_len = 0;
static uint32_t g_adc_dma_sample_rate_hz = ADC_DMA_SAMPLE_RATE_HZ;
static volatile uint16_t g_adc_dma_snapshot_len = 0;
static volatile uint8_t g_adc_dma_started = 0;
static uint32_t g_adc_acquisition_id = 0U;
static uint32_t g_adc_sequence = 0U;
static adc_dma_snapshot_info_t g_adc_snapshot_info;

volatile uint8_t g_adc_dma_sta = 0;     /* DMA传输状态标志, 0,未完成; 1, 已完成 */

static uint32_t adc_tim6_clock_hz(void)
{
    uint32_t tim_clk = HAL_RCC_GetPCLK1Freq();

    if ((RCC->D2CFGR & RCC_D2CFGR_D2PPRE1) != RCC_D2CFGR_D2PPRE1_DIV1)
    {
        tim_clk *= 2U;
    }

    return tim_clk;
}

static void adc_tim6_trgo_init(void)
{
    uint32_t tim_clk;
    uint32_t period;
    TIM_MasterConfigTypeDef master_config = {0};

    __HAL_RCC_TIM6_CLK_ENABLE();

    tim_clk = adc_tim6_clock_hz();
    period = (tim_clk + (ADC_DMA_SAMPLE_RATE_HZ / 2U)) / ADC_DMA_SAMPLE_RATE_HZ;
    if (period == 0U)
    {
        period = 1U;
    }
    g_adc_dma_sample_rate_hz = tim_clk / period;

    g_adc_tim_handle.Instance = TIM6;
    g_adc_tim_handle.Init.Prescaler = 0U;
    g_adc_tim_handle.Init.CounterMode = TIM_COUNTERMODE_UP;
    g_adc_tim_handle.Init.Period = period - 1U;
    g_adc_tim_handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    g_adc_tim_handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_Base_Init(&g_adc_tim_handle);

    master_config.MasterOutputTrigger = TIM_TRGO_UPDATE;
    master_config.MasterOutputTrigger2 = TIM_TRGO2_RESET;
    master_config.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&g_adc_tim_handle, &master_config);
}

/**
 * @brief       ADC DMA读取 初始化函数
 * @param       par         : 外设地址
 * @param       mar         : 存储器地址
 * @retval      无
 */
void adc_dma_init(uint32_t par, uint32_t mar)
{
    GPIO_InitTypeDef gpio_init_struct;
    ADC_ChannelConfTypeDef adc_ch_conf = {0};

    (void)par;

    g_adc_dma_mem = (uint16_t *)mar;
    g_adc_dma_len = 0U;
    g_adc_dma_snapshot_len = 0U;
    g_adc_dma_started = 0U;
    g_adc_acquisition_id = 0U;
    g_adc_sequence = 0U;
    g_adc_dma_sta = 0U;

    ADC_ADCX_CHY_GPIO_CLK_ENABLE();                                             /* 开启ADC通道IO引脚时钟 */
    ADC_ADCX_CHY_CLK_ENABLE();                                                  /* 使能ADC1/2时钟 */

    if ((uint32_t)ADC_ADCX_DMASx > (uint32_t)DMA2)                              /* 得到当前stream是属于DMA2还是DMA1 */
    {
        __HAL_RCC_DMA2_CLK_ENABLE();                                            /* DMA2时钟使能 */
    }
    else 
    {
        __HAL_RCC_DMA1_CLK_ENABLE();                                            /* DMA1时钟使能 */
    }

    __HAL_RCC_ADC_CONFIG(RCC_ADCCLKSOURCE_CLKP);                                /* ADC外设时钟选择 */

    /* 初始化ADC采集通道对应的IO引脚 */
    gpio_init_struct.Pin = ADC_ADCX_CHY_GPIO_PIN;                               /* ADC通道IO引脚 */
    gpio_init_struct.Mode = GPIO_MODE_ANALOG;                                   /* 模拟 */
    HAL_GPIO_Init(ADC_ADCX_CHY_GPIO_PORT, &gpio_init_struct);

    /* 初始化DMA */
    g_dma_adc_handle.Instance = ADC_ADCX_DMASx;                                 /* 设置DMA数据流 */
    g_dma_adc_handle.Init.Request = ADC_ADCX_DMASx_REQ;                         /* 请求选择DMA_REQUEST_ADC1 */
    g_dma_adc_handle.Init.Direction = DMA_PERIPH_TO_MEMORY;                     /* DIR = 1,外设到存储器模式 */
    g_dma_adc_handle.Init.PeriphInc = DMA_PINC_DISABLE;                         /* 外设非增量模式 */
    g_dma_adc_handle.Init.MemInc = DMA_MINC_ENABLE;                             /* 存储器增量模式 */
    g_dma_adc_handle.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;        /* 外设数据长度:16位 */
    g_dma_adc_handle.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;           /* 存储器数据长度:16位 */
    g_dma_adc_handle.Init.Mode = DMA_CIRCULAR;                                    /* 外设流控模式 */
    g_dma_adc_handle.Init.Priority = DMA_PRIORITY_MEDIUM;                       /* 中等优先级 */
    g_dma_adc_handle.Init.FIFOMode = DMA_FIFOMODE_DISABLE;                      /* 禁止FIFO*/
    HAL_DMA_Init(&g_dma_adc_handle);                                            /* 初始化DMA */

    __HAL_LINKDMA(&g_adc_dma_handle, DMA_Handle, g_dma_adc_handle);             /* 将DMA句柄与ADC句柄关联起来 */

    /* 初始化ADC */
    g_adc_dma_handle.Instance = ADC_ADCX;
    g_adc_dma_handle.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV2;                        /* 输入时钟2分频,即adc_ker_ck=per_ck/2=32Mhz */
    g_adc_dma_handle.Init.Resolution = ADC_RESOLUTION_16B;                              /* 16位模式 */
    g_adc_dma_handle.Init.ScanConvMode = DISABLE;                                       /* 非扫描模式 */
    g_adc_dma_handle.Init.EOCSelection = ADC_EOC_SINGLE_CONV;                           /* 关闭EOC中断 */
    g_adc_dma_handle.Init.LowPowerAutoWait = DISABLE;                                   /* 自动低功耗关闭 */
    g_adc_dma_handle.Init.ContinuousConvMode = DISABLE;                                  /* 开启连续转换 */
    g_adc_dma_handle.Init.NbrOfConversion = 1;                                          /* 1个转换在规则序列中 也就是只转换规则序列1 */
    g_adc_dma_handle.Init.DiscontinuousConvMode = DISABLE;                              /* 禁止不连续采样模式 */
    g_adc_dma_handle.Init.NbrOfDiscConversion = 0;                                      /* 不连续采样通道数为0 */
    g_adc_dma_handle.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T6_TRGO;                        /* 软件触发 */
    g_adc_dma_handle.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;         /* 使用软件触发 */
    g_adc_dma_handle.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;                           /* 有新的数据时直接覆盖掉旧数据 */
    g_adc_dma_handle.Init.OversamplingMode = DISABLE;                                   /* 过采样关闭 */
    g_adc_dma_handle.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;                         /* 设置ADC转换结果的左移位数 */
    g_adc_dma_handle.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR;    /* DMA单次传输ADC数据 */
    HAL_ADC_Init(&g_adc_dma_handle);                                                    /* 初始化 */

    HAL_ADCEx_Calibration_Start(&g_adc_dma_handle, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED); /* ADC校准 */

    /* 配置ADC通道 */
    adc_ch_conf.Channel = ADC_ADCX_CHY;                                         /* 配置使用的ADC通道 */
    adc_ch_conf.Rank = ADC_REGULAR_RANK_1;                                      /* 采样序列里的第1个 */
    adc_ch_conf.SamplingTime = ADC_SAMPLETIME_64CYCLES_5;                      /* 采样周期为387.5个时钟周期 */
    adc_ch_conf.SingleDiff = ADC_SINGLE_ENDED ;                                 /* 单端输入 */
    adc_ch_conf.OffsetNumber = ADC_OFFSET_NONE;                                 /* 无偏移 */
    adc_ch_conf.Offset = 0;                                                     /* 无偏移的情况下，此参数忽略 */
    adc_ch_conf.OffsetRightShift = DISABLE;                                     /* 禁止右移 */
    adc_ch_conf.OffsetSignedSaturation = DISABLE;                               /* 禁止有符号饱和 */
    HAL_ADC_ConfigChannel(&g_adc_dma_handle, &adc_ch_conf);                     /* 配置ADC通道 */

    /* 配置DMA数据流请求中断优先级 */
    HAL_NVIC_SetPriority(ADC_ADCX_DMASx_IRQn, 3, 3);                            /* 设置DMA中断优先级为3，子优先级为3 */
    HAL_NVIC_EnableIRQ(ADC_ADCX_DMASx_IRQn);                                    /* 使能DMA中断 */
    
    adc_tim6_trgo_init();
}

/**
 * @brief       使能一次ADC DMA传输 
 * @param       ndtr: DMA传输的次数
 * @retval      无
 */
void adc_dma_enable(uint16_t ndtr)
{
    if (g_adc_dma_started || g_adc_dma_mem == NULL || ndtr == 0U || ndtr > ADC_DMA_MAX_SAMPLES)
    {
        return;
    }

    g_adc_dma_len = ndtr;
    (void)adc_dma_resume();
}

bool adc_dma_pause(void)
{
    HAL_StatusTypeDef adc_status = HAL_OK;
    HAL_StatusTypeDef tim_status;

    HAL_NVIC_DisableIRQ(ADC_ADCX_DMASx_IRQn);
    tim_status = HAL_TIM_Base_Stop(&g_adc_tim_handle);
    if (g_adc_dma_started != 0U)
    {
        g_adc_dma_started = 0U; /* No more snapshot publication. */
        adc_status = HAL_ADC_Stop_DMA(&g_adc_dma_handle);
    }
    g_adc_dma_sta = 0U;
    g_adc_dma_snapshot_len = 0U;
    HAL_NVIC_ClearPendingIRQ(ADC_ADCX_DMASx_IRQn);
    return tim_status == HAL_OK && adc_status == HAL_OK;
}

bool adc_dma_resume(void)
{
    if (g_adc_dma_started != 0U) { return true; }
    if (g_adc_dma_mem == NULL || g_adc_dma_len == 0U) { return false; }

    HAL_NVIC_DisableIRQ(ADC_ADCX_DMASx_IRQn);
    g_adc_dma_sta = 0U;
    g_adc_dma_snapshot_len = 0U;
    HAL_NVIC_ClearPendingIRQ(ADC_ADCX_DMASx_IRQn);
    __HAL_TIM_SET_COUNTER(&g_adc_tim_handle, 0U);
    __HAL_TIM_CLEAR_FLAG(&g_adc_tim_handle, TIM_FLAG_UPDATE);

    /* DMA is stopped here. Discard cached lines before the new DMA writes. */
    uint32_t bytes = ((uint32_t)g_adc_dma_len * sizeof(uint16_t) + 31U) & ~31U;
    SCB_InvalidateDCache_by_Addr((uint32_t *)g_adc_dma_mem, bytes);
    if (HAL_ADC_Start_DMA(&g_adc_dma_handle, (uint32_t *)g_adc_dma_mem, g_adc_dma_len) != HAL_OK)
    {
        (void)HAL_ADC_Stop_DMA(&g_adc_dma_handle);
        return false;
    }

    __HAL_DMA_DISABLE_IT(&g_dma_adc_handle, DMA_IT_HT);
    g_adc_acquisition_id++;
    g_adc_dma_started = 1U;
    HAL_NVIC_EnableIRQ(ADC_ADCX_DMASx_IRQn);

    if (HAL_TIM_Base_Start(&g_adc_tim_handle) != HAL_OK)
    {
        g_adc_dma_started = 0U;
        HAL_NVIC_DisableIRQ(ADC_ADCX_DMASx_IRQn);
        (void)HAL_ADC_Stop_DMA(&g_adc_dma_handle);
        return false;
    }
    return true;
}

bool adc_dma_is_running(void)
{
    return g_adc_dma_started != 0U;
}

uint32_t adc_dma_get_acquisition_id(void)
{
    return g_adc_acquisition_id;
}


/**
 * @brief       ADC DMA采集中断服务函数
 * @param       无
 * @retval      无
 */
void ADC_ADCX_DMASx_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&g_dma_adc_handle);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc == &g_adc_dma_handle && g_adc_dma_started != 0U)
    {
        if (g_adc_dma_mem != NULL && g_adc_dma_len > 0U)
        {
            uint32_t bytes = ((uint32_t)g_adc_dma_len * sizeof(uint16_t) + 31U) & ~31U;
            SCB_InvalidateDCache_by_Addr((uint32_t *)g_adc_dma_mem, bytes);
            memcpy(g_adc_dma_snapshot, g_adc_dma_mem, (uint32_t)g_adc_dma_len * sizeof(uint16_t));
            g_adc_dma_snapshot_len = g_adc_dma_len;
            g_adc_snapshot_info.completed_ms = HAL_GetTick();
            g_adc_snapshot_info.sequence = ++g_adc_sequence;
            g_adc_snapshot_info.acquisition_id = g_adc_acquisition_id;
            g_adc_dma_sta = 1U;
        }
    }
}

uint16_t adc_dma_read_snapshot(uint16_t *dst, uint16_t max_count)
{
    return adc_dma_read_snapshot_ex(dst, max_count, NULL);
}

uint16_t adc_dma_read_snapshot_ex(uint16_t *dst, uint16_t max_count,
                                  adc_dma_snapshot_info_t *info)
{
    uint16_t count;
    uint32_t primask;

    if (dst == NULL || max_count == 0U || g_adc_dma_sta == 0U)
    {
        return 0U;
    }

    primask = __get_PRIMASK();
    __disable_irq();

    count = g_adc_dma_snapshot_len;
    if (count > max_count)
    {
        count = max_count;
    }

    memcpy(dst, g_adc_dma_snapshot, (uint32_t)count * sizeof(uint16_t));
    if (info != NULL) { *info = g_adc_snapshot_info; }
    g_adc_dma_sta = 0U;

    if (primask == 0U)
    {
        __enable_irq();
    }

    return count;
}

uint32_t adc_dma_get_sample_rate_hz(void)
{
    return g_adc_dma_sample_rate_hz;
}


