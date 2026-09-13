#ifndef POWER_METER_FAKE_PORT_H
#define POWER_METER_FAKE_PORT_H
#include "power_meter.h"

typedef struct
{
    uint32_t now_ms, acquisition_id, generation, sequence;
    uint32_t pauses, resumes, switches;
    pm_range_t range;
    bool running, pause_ok, resume_ok, switch_ok;
} pm_fake_port_t;
extern pm_fake_port_t pm_fake;
void pm_test_reset(pm_range_t range);
bool pm_test_submit(const uint16_t *samples, uint16_t count, pm_range_t range);
void pm_test_manual_switch(pm_range_t range);
#endif
