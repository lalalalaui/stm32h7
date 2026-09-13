#include "power_meter.h"
#include "power_meter_config.h"
#include "power_meter_fake_port.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static void near_value(float actual, float expected)
{
    assert(fabsf(actual - expected) < 0.00001f);
}

int main(void)
{
    const uint16_t zero[] = {0U, 0U};
    const uint16_t full[] = {65535U, 65535U};
    const uint16_t alternating[] = {0U, 65535U, 0U, 65535U};
    static uint16_t largest_block[UINT16_MAX];
    const pm_measurement_t *m = PM_GetMeasurement();

    _Static_assert(PM_ADC_FULL_SCALE == 65535UL, "Stage 2 must retain 16-bit conversion");
    pm_test_reset(PM_RANGE_LOW);
    assert(PM_SetMode(PM_MODE_MANUAL));
    assert(!m->valid && m->current_range == PM_RANGE_LOW);
    assert(pm_test_submit(zero, 2U, PM_RANGE_LOW));
    near_value(m->adc_voltage_V, 0.0f);
    near_value(m->filtered_voltage_V, 0.0f);

    /* Full scale must be 3.3 V, not a 12-bit-scaled voltage. */
    assert(pm_test_submit(full, 2U, PM_RANGE_LOW));
    near_value(m->adc_voltage_V, 3.3f);
    near_value(m->filtered_voltage_V, 0.825f);
    assert(pm_test_submit(full, 2U, PM_RANGE_LOW));
    near_value(m->filtered_voltage_V, 1.44375f);

    /* First block is seeded directly and fractional ADC averages survive. */
    pm_test_reset(PM_RANGE_LOW);
    assert(pm_test_submit(alternating, 4U, PM_RANGE_LOW));
    assert(m->adc_raw == 65535U && m->adc_min == 0U && m->adc_max == 65535U);
    assert(m->sample_count == 4U && m->processed_blocks == 1U && m->valid);
    near_value(m->adc_mean, 32767.5f);
    near_value(m->adc_voltage_V, 1.65f);
    near_value(m->filtered_voltage_V, 1.65f);

    /* Manual range changes must not retain the previous range's IIR value. */
    pm_test_manual_switch(PM_RANGE_HIGH);
    assert(pm_test_submit(full, 2U, PM_RANGE_HIGH));
    assert(m->current_range == PM_RANGE_HIGH);
    near_value(m->filtered_voltage_V, 3.3f);
    pm_test_manual_switch(PM_RANGE_MID);
    assert(pm_test_submit(zero, 2U, PM_RANGE_MID));
    assert(m->current_range == PM_RANGE_MID);
    near_value(m->filtered_voltage_V, 0.0f);

    /* Bad input is rejected without partially publishing a new measurement. */
    pm_measurement_t saved = *m;
    assert(!pm_test_submit(NULL, 2U, PM_RANGE_MID));
    assert(!pm_test_submit(full, 0U, PM_RANGE_MID));
    assert(!pm_test_submit(full, 2U, (pm_range_t)99));
    assert(!PM_SetVDDA(0.0f));
    assert(!PM_SetVDDA(-1.0f));
    assert(!PM_SetVDDA(NAN));
    assert(!PM_SetVDDA(INFINITY));
    assert(!PM_SetVDDA(5.0f));
    assert(memcmp(&saved, m, sizeof(saved)) == 0);

    assert(PM_SetVDDA(3.0f));
    assert(!m->valid);
    assert(pm_test_submit(full, 2U, PM_RANGE_MID));
    near_value(m->adc_voltage_V, 3.0f);
    near_value(m->filtered_voltage_V, 3.0f);

    /* Largest permitted input length exercises accumulator overflow bounds. */
    for (uint32_t i = 0U; i < UINT16_MAX; i++) { largest_block[i] = UINT16_MAX; }
    pm_test_manual_switch(PM_RANGE_LOW);
    assert(pm_test_submit(largest_block, UINT16_MAX, PM_RANGE_LOW));
    near_value(m->adc_mean, 65535.0f);
    near_value(m->adc_voltage_V, 3.0f);
    assert(m->adc_min == UINT16_MAX && m->adc_max == UINT16_MAX);

    puts("PASS: ADC statistics, 16-bit scaling, IIR, range reset, VDDA, invalid input and maximum block.");
    return 0;
}
