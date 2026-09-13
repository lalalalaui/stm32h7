#include "power_meter_port.h"
#include "./BSP/ADC/adc.h"

uint32_t PM_PortNowMs(void) { return HAL_GetTick(); }
bool PM_PortPauseADC(void) { return adc_dma_pause(); }
bool PM_PortResumeADC(void) { return adc_dma_resume(); }
bool PM_PortADCIsRunning(void) { return adc_dma_is_running(); }
uint32_t PM_PortAcquisitionId(void) { return adc_dma_get_acquisition_id(); }
bool PM_PortSetRange(pm_range_t range) { return PM_HW_SetRange(range); }
pm_range_t PM_PortGetRange(void) { return PM_HW_GetRange(); }
uint32_t PM_PortRangeGeneration(void) { return PM_HW_GetRangeGeneration(); }
