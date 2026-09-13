#include "power_meter_hw.h"
#include "power_meter_board.h"
#include "./SYSTEM/delay/delay.h"

static pm_range_t g_pm_range = PM_RANGE_LOW;
static bool g_pm_initialized = false;
static uint32_t g_pm_range_generation = 0U;

void PM_HW_Init(void)
{
    GPIO_InitTypeDef gpio_init = {0};

    PM_GAIN_LOW_GPIO_CLK_ENABLE();
    PM_GAIN_MID_GPIO_CLK_ENABLE();

    /* Match the external LOW pullup / MID pulldown before output enable. */
    HAL_GPIO_WritePin(PM_GAIN_LOW_GPIO_Port, PM_GAIN_LOW_Pin, PM_GAIN_LOW_BOOT_STATE);
    HAL_GPIO_WritePin(PM_GAIN_MID_GPIO_Port, PM_GAIN_MID_Pin, PM_GAIN_MID_BOOT_STATE);

    gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_init.Pull = GPIO_NOPULL;
    gpio_init.Speed = GPIO_SPEED_FREQ_LOW;
    gpio_init.Pin = PM_GAIN_LOW_Pin;
    HAL_GPIO_Init(PM_GAIN_LOW_GPIO_Port, &gpio_init);
    gpio_init.Pin = PM_GAIN_MID_Pin;
    HAL_GPIO_Init(PM_GAIN_MID_GPIO_Port, &gpio_init);

    g_pm_range = PM_RANGE_LOW;
    g_pm_initialized = true;
    g_pm_range_generation++;
}

bool PM_HW_SetRange(pm_range_t range)
{
    uint32_t primask;

    if (!g_pm_initialized ||
        (range != PM_RANGE_HIGH && range != PM_RANGE_MID && range != PM_RANGE_LOW))
    {
        return false;
    }

    if (range == g_pm_range)
    {
        return true;
    }

    /* Keep the temporary 1/1 overlap from being extended by an ISR.
     * SysTick still counts during the 1 us delay; no millisecond wait is used.
     */
    primask = __get_PRIMASK();
    __disable_irq();

    switch (range)
    {
        case PM_RANGE_HIGH:
            HAL_GPIO_WritePin(PM_GAIN_MID_GPIO_Port, PM_GAIN_MID_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(PM_GAIN_LOW_GPIO_Port, PM_GAIN_LOW_Pin, GPIO_PIN_RESET);
            break;

        case PM_RANGE_MID:
            HAL_GPIO_WritePin(PM_GAIN_MID_GPIO_Port, PM_GAIN_MID_Pin, GPIO_PIN_SET);
            __DSB();
            if (g_pm_range == PM_RANGE_LOW)
            {
                delay_us(PM_GAIN_OVERLAP_US);
            }
            HAL_GPIO_WritePin(PM_GAIN_LOW_GPIO_Port, PM_GAIN_LOW_Pin, GPIO_PIN_RESET);
            break;

        case PM_RANGE_LOW:
            HAL_GPIO_WritePin(PM_GAIN_LOW_GPIO_Port, PM_GAIN_LOW_Pin, GPIO_PIN_SET);
            __DSB();
            if (g_pm_range == PM_RANGE_MID)
            {
                delay_us(PM_GAIN_OVERLAP_US);
            }
            HAL_GPIO_WritePin(PM_GAIN_MID_GPIO_Port, PM_GAIN_MID_Pin, GPIO_PIN_RESET);
            break;

        default:
            break; /* Invalid values were rejected before disabling IRQs. */
    }

    __DSB();
    g_pm_range = range;
    g_pm_range_generation++;
    __set_PRIMASK(primask);
    return true;
}

pm_range_t PM_HW_GetRange(void)
{
    return g_pm_range;
}

uint32_t PM_HW_GetRangeGeneration(void)
{
    return g_pm_range_generation;
}
