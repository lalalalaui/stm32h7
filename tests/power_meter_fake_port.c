#include "power_meter_fake_port.h"
#include "power_meter_port.h"
#include "power_meter_config.h"
#include <assert.h>

pm_fake_port_t pm_fake;
uint32_t PM_PortNowMs(void) { return pm_fake.now_ms; }
bool PM_PortPauseADC(void)
{
    pm_fake.pauses++;
    if (!pm_fake.pause_ok) { return false; }
    pm_fake.running = false;
    return true;
}
bool PM_PortResumeADC(void)
{
    assert(!pm_fake.running);
    pm_fake.resumes++;
    if (!pm_fake.resume_ok) { return false; }
    pm_fake.running = true;
    pm_fake.acquisition_id++;
    return true;
}
bool PM_PortADCIsRunning(void) { return pm_fake.running; }
uint32_t PM_PortAcquisitionId(void) { return pm_fake.acquisition_id; }
bool PM_PortSetRange(pm_range_t range)
{
    /* Every controller-driven GPIO transition MUST be behind the DMA fence. */
    assert(!pm_fake.running);
    pm_fake.switches++;
    if (!pm_fake.switch_ok) { return false; }
    pm_fake.range = range;
    pm_fake.generation++;
    return true;
}
pm_range_t PM_PortGetRange(void) { return pm_fake.range; }
uint32_t PM_PortRangeGeneration(void) { return pm_fake.generation; }

void pm_test_reset(pm_range_t range)
{
    pm_fake = (pm_fake_port_t){.range = range, .acquisition_id = 1U, .generation = 1U,
        .running = true, .pause_ok = true, .resume_ok = true, .switch_ok = true};
    PM_Init(range);
    assert(PM_Service());
}

bool pm_test_submit(const uint16_t *samples, uint16_t count, pm_range_t range)
{
    pm_sample_info_t info = {.completed_ms = pm_fake.now_ms, .sequence = ++pm_fake.sequence,
        .acquisition_id = pm_fake.acquisition_id, .range = range};
    return PM_ProcessADCSamples(samples, count, &info);
}

void pm_test_manual_switch(pm_range_t range)
{
    pm_range_t previous = pm_fake.range;
    assert(PM_SetManualRange(range));
    if (previous != range)
    {
        assert(!PM_Service());
        assert(!pm_fake.running);
        pm_fake.now_ms += PM_RANGE_SETTLE_MS;
        assert(!PM_Service());
        pm_fake.now_ms++;
        assert(PM_Service());
        assert(pm_fake.running && pm_fake.range == range);
        assert(!PM_GetMeasurement()->valid);
    }
    else { assert(PM_Service()); }
}
