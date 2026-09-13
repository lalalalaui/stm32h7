#ifndef POWER_METER_HW_H
#define POWER_METER_HW_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    PM_RANGE_HIGH = 0, /* GAIN_MID=0, GAIN_LOW=0; high gain. */
    PM_RANGE_MID,      /* GAIN_MID=1, GAIN_LOW=0. */
    PM_RANGE_LOW       /* GAIN_MID=0, GAIN_LOW=1; low gain. */
} pm_range_t;

/* Call once immediately after HAL_Init(). No delay_init() dependency.
 * Preloads LOW levels before enabling outputs; leaves the ADC BSP in charge
 * of configuring the AFE_OUT analog input and starting acquisition.
 */
void PM_HW_Init(void);

/* Driver-level control, called from the main loop after delay_init().
 * Application/UI code should use PM_SetManualRange() so ADC fencing is applied.
 * Returns false before PM_HW_Init() or for an invalid range, without changing
 * outputs. MID <-> LOW uses a brief make-before-break overlap.
 * Returns after GPIO switching, NOT after analog settling. Existing ADC/DMA
 * acquisition continues, so samples near a manual transition are transients.
 */
bool PM_HW_SetRange(pm_range_t range);

/* Last commanded stable range; meaningful after PM_HW_Init(). */
pm_range_t PM_HW_GetRange(void);
uint32_t PM_HW_GetRangeGeneration(void);

#ifdef __cplusplus
}
#endif

#endif /* POWER_METER_HW_H */
