#ifndef POWER_METER_PORT_H
#define POWER_METER_PORT_H

/* Hardware adapter; these functions are replaced by fakes in host tests. */
#include "power_meter_hw.h"

uint32_t PM_PortNowMs(void);
bool PM_PortPauseADC(void);
bool PM_PortResumeADC(void);
bool PM_PortADCIsRunning(void);
uint32_t PM_PortAcquisitionId(void);
bool PM_PortSetRange(pm_range_t range);
pm_range_t PM_PortGetRange(void);
uint32_t PM_PortRangeGeneration(void);

#endif
