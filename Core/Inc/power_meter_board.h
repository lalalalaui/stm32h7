#ifndef POWER_METER_BOARD_H
#define POWER_METER_BOARD_H

#include "stm32h7xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/* AFE wiring for touchh7 / STM32H743IIT6.
 * This is the single source of truth for the power-meter signal pins.
 * PA5 already belongs to the ADC BSP; PB8/PB9 are unused by the current BSPs.
 * Check development-board connector routing before wiring the AFE.
 */
#define PM_AFE_OUT_GPIO_Port                GPIOA
#define PM_AFE_OUT_Pin                      GPIO_PIN_5
#define PM_AFE_OUT_GPIO_CLK_ENABLE()        __HAL_RCC_GPIOA_CLK_ENABLE()
#define PM_ADC_INSTANCE                     ADC1
#define PM_ADC_CHANNEL                      ADC_CHANNEL_19
#define PM_ADC_CLK_ENABLE()                 __HAL_RCC_ADC12_CLK_ENABLE()

/* ADG721 controls: logic 1 connects the corresponding feedback branch.
 * HIGH: MID=0 LOW=0; MID: MID=1 LOW=0; LOW: MID=0 LOW=1.
 * The AFE has a 100 kohm MID pulldown and a 100 kohm LOW pullup.
 */
#define PM_GAIN_MID_GPIO_Port               GPIOB
#define PM_GAIN_MID_Pin                     GPIO_PIN_8
#define PM_GAIN_MID_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOB_CLK_ENABLE()
#define PM_GAIN_LOW_GPIO_Port               GPIOB
#define PM_GAIN_LOW_Pin                     GPIO_PIN_9
#define PM_GAIN_LOW_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOB_CLK_ENABLE()

#define PM_GAIN_MID_BOOT_STATE              GPIO_PIN_RESET
#define PM_GAIN_LOW_BOOT_STATE              GPIO_PIN_SET

/* Brief make-before-break overlap when switching MID <-> LOW. */
#define PM_GAIN_OVERLAP_US                  1U

#ifdef __cplusplus
}
#endif

#endif /* POWER_METER_BOARD_H */
