#ifndef POWER_METER_STORAGE_H
#define POWER_METER_STORAGE_H

#include <stdbool.h>
#include <stdint.h>

/* AT24C02 allocation: touch calibration is 40..52 and EEPROM probe is 255.
 * This record occupies 64..103. No automatic/periodic EEPROM writes occur.
 */
#define PM_STORAGE_ADDR 64U
#define PM_STORAGE_SIZE 84U

typedef enum
{
    PM_DISPLAY_MODE_MW = 0,
    PM_DISPLAY_MODE_UW,
    PM_DISPLAY_MODE_DBM
} pm_display_mode_t;

/* Call after PM_Init() and touch initialization. A false return means no valid
 * record; PM defaults remain active. On success it restores PM parameters,
 * wavelength and AUTO/MANUAL state. Manual startup remains safe LOW range.
 */
bool PM_Storage_Load(pm_display_mode_t *display_mode);

/* Explicit user action only: ZERO completion, CAL completion, or 保存设置. */
bool PM_Storage_Save(pm_display_mode_t display_mode);

#endif /* POWER_METER_STORAGE_H */
