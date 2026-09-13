/**
 ****************************************************************************************************
 * @file        adc.h
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
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
 *
 ****************************************************************************************************
 */

#ifndef __ADC_H
#define __ADC_H

#include "./SYSTEM/sys/sys.h"
#include "power_meter_board.h"
#include <stdbool.h>

/******************************************************************************************/

/* ADC及引脚 定义 */
#define ADC_ADCX_CHY_GPIO_PORT              PM_AFE_OUT_GPIO_Port
#define ADC_ADCX_CHY_GPIO_PIN               PM_AFE_OUT_Pin
#define ADC_ADCX_CHY_GPIO_CLK_ENABLE()      do{ PM_AFE_OUT_GPIO_CLK_ENABLE(); }while(0)

#define ADC_ADCX                            PM_ADC_INSTANCE
#define ADC_ADCX_CHY                        PM_ADC_CHANNEL
#define ADC_ADCX_CHY_CLK_ENABLE()           do{ PM_ADC_CLK_ENABLE(); }while(0)

/* ADC单通道/多通道 DMA采集 DMA数据流相关 定义
 * 注意: 这里我们的通道还是使用上面的定义.
 */
#define ADC_ADCX_DMASx                      DMA2_Stream4
#define ADC_ADCX_DMASx_REQ                  DMA_REQUEST_ADC1                        /* ADC1_DMA请求源 */
#define ADC_ADCX_DMASx_IRQn                 DMA2_Stream4_IRQn
#define ADC_ADCX_DMASx_IRQHandler           DMA2_Stream4_IRQHandler

#define ADC_ADCX_DMASx_IS_TC()              ( DMA2->HISR & (1 << 5) )               /* 判断 DMA2_Stream4 传输完成标志, 这是一个假函数形式,
                                                                                     * 不能当函数使用, 只能用在if等语句里面 
                                                                                     */
#define ADC_ADCX_DMASx_CLR_TC()             do{ DMA2->HIFCR |= 1 << 5; }while(0)    /* 清除 DMA2_Stream4 传输完成标志 */

/******************************************************************************************/

#define ADC_DMA_SAMPLE_RATE_HZ                               24000U
#define ADC_DMA_MAX_SAMPLES                                  1024U

void adc_init(void);                                          /* ADC通道初始化 */
uint32_t adc_get_result(uint32_t ch);                         /* 获得某个通道值  */
uint32_t adc_get_result_average(uint32_t ch, uint8_t times);  /* 得到某个通道给定次数采样的平均值 */

extern volatile uint8_t g_adc_dma_sta;                        /* DMA传输完成标志 */
void adc_dma_init(uint32_t par, uint32_t mar);                /* ADC DMA采集初始化 */
void adc_dma_enable( uint16_t ndtr);                          /* 使能一次ADC DMA采集传输 */
uint16_t adc_dma_read_snapshot(uint16_t *dst, uint16_t max_count);
uint32_t adc_dma_get_sample_rate_hz(void);

typedef struct
{
    uint32_t completed_ms;
    uint32_t sequence;
    uint32_t acquisition_id;
} adc_dma_snapshot_info_t;

/* Main-loop-only pause/resume. Resume starts a NEW full buffer, same rate. */
bool adc_dma_pause(void);
bool adc_dma_resume(void);
bool adc_dma_is_running(void);
uint32_t adc_dma_get_acquisition_id(void);
uint16_t adc_dma_read_snapshot_ex(uint16_t *dst, uint16_t max_count,
                                  adc_dma_snapshot_info_t *info);

#endif 
