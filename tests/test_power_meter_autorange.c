#include "power_meter_fake_port.h"
#include "power_meter_config.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static const pm_measurement_t *m;

static void feed(float voltage_V, uint32_t step_ms)
{
    uint16_t samples[80];
    uint16_t raw = (uint16_t)(voltage_V / 3.3f * 65535.0f + 0.5f);
    for (unsigned i = 0; i < 80; i++) { samples[i] = raw; }
    pm_fake.now_ms += step_ms;
    assert(pm_test_submit(samples, 80U, pm_fake.range));
}

static void finish_switch(pm_range_t range)
{
    uint32_t old_id = pm_fake.acquisition_id;
    uint16_t old_samples[] = {60000U, 60000U};
    pm_sample_info_t old_info = {.completed_ms = pm_fake.now_ms, .sequence = ++pm_fake.sequence,
        .acquisition_id = old_id, .range = range};
    assert(m->target_range == range);
    assert(!PM_Service());
    assert(pm_fake.range == range && !pm_fake.running);
    assert(m->range_switching && !m->valid);
    assert(!PM_ProcessADCSamples(old_samples, 2U, &old_info));
    pm_fake.now_ms += 2U;
    assert(!PM_Service());
    pm_fake.now_ms++;
    assert(PM_Service());
    assert(pm_fake.acquisition_id != old_id && pm_fake.running);
    assert(m->range_switching && !m->valid);
    /* Matching range alone is insufficient: old acquisition ID is rejected. */
    old_info.completed_ms = pm_fake.now_ms;
    assert(!PM_ProcessADCSamples(old_samples, 2U, &old_info));
    feed(0.9f, 4U);
    assert(m->valid && !m->range_switching);
    assert(fabsf(m->filtered_voltage_V - m->adc_voltage_V) < 0.00001f);
    assert(fabsf(m->adc_voltage_V - 0.9f) < 0.0001f);
}

int main(void)
{
    m = PM_GetMeasurement();
    pm_test_reset(PM_RANGE_HIGH);
    assert(m->mode == PM_MODE_AUTO);
    feed(2.5f, 0U);
    feed(2.5f, 3U);
    assert(m->target_range == PM_RANGE_HIGH);
    feed(2.5f, 3U);
    finish_switch(PM_RANGE_MID);
    feed(2.5f, 4U); feed(2.5f, 3U); feed(2.5f, 3U);
    finish_switch(PM_RANGE_LOW);

    for (unsigned i = 0; i < 20; i++) { feed(0.1f, 4U); }
    assert(m->target_range == PM_RANGE_LOW); /* 76 ms from first low frame. */
    feed(0.1f, 4U);
    finish_switch(PM_RANGE_MID);
    for (unsigned i = 0; i < 21; i++) { feed(0.1f, 4U); }
    finish_switch(PM_RANGE_HIGH);

    /* Boundary ranges never wrap beyond HIGH/LOW. */
    for (unsigned i = 0; i < 50; i++) { feed(0.01f, 4U); }
    assert(m->target_range == PM_RANGE_HIGH);
    pm_test_reset(PM_RANGE_LOW);
    feed(3.1f, 4U);
    assert(m->over_range && m->target_range == PM_RANGE_LOW);
    feed(2.95f, 4U); assert(m->over_range);
    feed(2.8f, 4U); assert(!m->over_range);

    /* Noise around either threshold resets the time confirmation. */
    pm_test_reset(PM_RANGE_MID);
    for (unsigned i = 0; i < 50; i++) { feed(2.41f, 4U); feed(2.39f, 4U); }
    assert(m->target_range == PM_RANGE_MID);
    for (unsigned i = 0; i < 50; i++) { feed(0.149f, 4U); feed(0.151f, 4U); }
    assert(m->target_range == PM_RANGE_MID);

    /* A missing block or a long sampling gap cannot satisfy the 80 ms hold. */
    pm_test_reset(PM_RANGE_LOW);
    for (unsigned i = 0; i < 20; i++) { feed(0.1f, 4U); }
    pm_fake.sequence++; /* Lost a DMA snapshot. */
    feed(0.1f, 4U);
    assert(m->target_range == PM_RANGE_LOW);
    feed(0.1f, 40U);
    assert(m->target_range == PM_RANGE_LOW);
    for (unsigned i = 0; i < 19; i++) { feed(0.1f, 4U); }
    assert(m->target_range == PM_RANGE_LOW);
    feed(0.1f, 4U);
    assert(m->target_range == PM_RANGE_MID);

    /* Every manual source/target combination, including no-op, remains usable. */
    for (unsigned from = 0; from < 3; from++) {
        for (unsigned to = 0; to < 3; to++) {
            pm_test_reset((pm_range_t)from);
            pm_test_manual_switch((pm_range_t)to);
            assert(m->mode == PM_MODE_MANUAL);
            for (unsigned i = 0; i < 25; i++) { feed(0.05f, 4U); }
            assert(m->target_range == (pm_range_t)to);
            feed(3.1f, 4U); feed(3.1f, 10U);
            assert(m->target_range == (pm_range_t)to);
        }
    }
    assert(!PM_SetMode((pm_mode_t)99));
    assert(!PM_SetManualRange((pm_range_t)99));

    /* Switching back to AUTO starts fresh confirmation, not a manual-era hold. */
    pm_test_reset(PM_RANGE_LOW);
    assert(PM_SetMode(PM_MODE_MANUAL));
    for (unsigned i = 0; i < 30; i++) { feed(0.1f, 4U); }
    assert(PM_SetMode(PM_MODE_AUTO));
    feed(0.1f, 4U);
    assert(m->target_range == PM_RANGE_LOW);
    for (unsigned i = 0; i < 20; i++) { feed(0.1f, 4U); }
    assert(m->target_range == PM_RANGE_MID);
    assert(PM_SetMode(PM_MODE_MANUAL)); /* Cancel an unexecuted auto request. */
    assert(PM_Service() && pm_fake.switches == 0U && pm_fake.range == PM_RANGE_LOW);

    /* A new manual target during settling is applied with ADC still stopped. */
    assert(PM_SetManualRange(PM_RANGE_MID));
    assert(!PM_Service());
    pm_fake.now_ms++;
    assert(PM_SetManualRange(PM_RANGE_HIGH));
    assert(!PM_Service() && !pm_fake.running && pm_fake.range == PM_RANGE_HIGH);
    pm_fake.now_ms += 2U;
    assert(!PM_Service());
    pm_fake.now_ms++;
    assert(PM_Service());
    feed(1.0f, 4U);
    assert(m->valid && m->mode == PM_MODE_MANUAL && m->current_range == PM_RANGE_HIGH);

    /* Millisecond and sequence rollover remain valid. */
    pm_test_reset(PM_RANGE_LOW);
    pm_fake.now_ms = UINT32_MAX - 40U;
    pm_fake.sequence = UINT32_MAX - 10U;
    for (unsigned i = 0; i < 21; i++) { feed(0.1f, 4U); }
    finish_switch(PM_RANGE_MID);

    /* Duplicated and stale data cannot advance confirmation or overwrite results. */
    pm_test_reset(PM_RANGE_LOW);
    feed(0.1f, 4U);
    uint16_t raw[] = {2000U};
    pm_sample_info_t info = {.completed_ms = pm_fake.now_ms, .sequence = pm_fake.sequence,
        .acquisition_id = pm_fake.acquisition_id, .range = PM_RANGE_LOW};
    uint32_t blocks = m->processed_blocks;
    assert(!PM_ProcessADCSamples(raw, 1U, &info));
    info.sequence++;
    pm_fake.now_ms += 81U;
    assert(!PM_ProcessADCSamples(raw, 1U, &info));
    assert(m->processed_blocks == blocks && m->target_range == PM_RANGE_LOW);

    /* Detect even direct legacy GPIO changes which return to the original range. */
    pm_test_reset(PM_RANGE_LOW);
    feed(1.0f, 4U);
    pm_fake.generation += 2U;
    finish_switch(PM_RANGE_LOW);
    assert(m->mode == PM_MODE_MANUAL);

    /* Hardware failures never publish the previous range as a valid new result. */
    pm_test_reset(PM_RANGE_LOW);
    assert(PM_SetManualRange(PM_RANGE_HIGH));
    pm_fake.pause_ok = false;
    assert(!PM_Service() && m->adc_fault && !m->valid);
    assert(pm_fake.switches == 0U);
    pm_test_reset(PM_RANGE_LOW);
    assert(PM_SetManualRange(PM_RANGE_HIGH));
    pm_fake.switch_ok = false;
    assert(!PM_Service() && m->adc_fault && !pm_fake.running);
    pm_test_reset(PM_RANGE_LOW);
    assert(PM_SetManualRange(PM_RANGE_HIGH));
    assert(!PM_Service());
    pm_fake.now_ms += 3U;
    pm_fake.resume_ok = false;
    assert(!PM_Service() && m->adc_fault && !m->valid);

    puts("PASS: auto/manual, hysteresis, hold times, all range transitions, settling, old epochs, rollover and faults.");
    return 0;
}
