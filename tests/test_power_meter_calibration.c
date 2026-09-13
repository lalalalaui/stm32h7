#include "power_meter_fake_port.h"
#include "power_meter_config.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static const pm_measurement_t *m;

static void near_value(float actual, float expected)
{
    assert(fabsf(actual - expected) < 0.00002f);
}

static void feed_raw(uint16_t raw)
{
    uint16_t samples[] = {raw, raw, raw, raw};
    pm_fake.now_ms += 4U;
    assert(pm_test_submit(samples, 4U, pm_fake.range));
    PM_CalibrationService();
}

static void service_to_range(pm_range_t range)
{
    assert(!PM_Service());
    assert(!pm_fake.running && pm_fake.range == range);
    pm_fake.now_ms += PM_RANGE_SETTLE_MS;
    assert(!PM_Service());
    pm_fake.now_ms++;
    assert(PM_Service() && pm_fake.running);
}

static void collect_zero_range(pm_range_t range, uint16_t raw)
{
    assert(pm_fake.range == range);
    /* First valid new-epoch block is deliberately discarded by ZERO. */
    feed_raw(raw);
    for (uint32_t i = 0U; i < PM_ZERO_BLOCKS_PER_RANGE; i++) { feed_raw(raw); }
}

static void test_zero_three_ranges_and_restore_manual(void)
{
    const uint16_t raw = 10000U;
    const float voltage = (float)raw / 65535.0f * 3.3f;
    pm_test_reset(PM_RANGE_MID);
    assert(PM_SetMode(PM_MODE_MANUAL));
    feed_raw(40000U);
    pm_power_parameters_t before = *PM_GetPowerParameters();
    assert(PM_SetRangeCorrection(PM_RANGE_HIGH, 0.1f, 1.1f));
    assert(PM_SetRangeCorrection(PM_RANGE_MID, 0.2f, 1.2f));
    assert(PM_SetRangeCorrection(PM_RANGE_LOW, 0.3f, 1.3f));
    assert(PM_StartZero());
    assert(PM_GetZeroState() == PM_ZERO_LOW);
    service_to_range(PM_RANGE_LOW);
    collect_zero_range(PM_RANGE_LOW, raw);
    assert(PM_GetZeroState() == PM_ZERO_MID);
    service_to_range(PM_RANGE_MID);
    collect_zero_range(PM_RANGE_MID, raw);
    assert(PM_GetZeroState() == PM_ZERO_HIGH);
    service_to_range(PM_RANGE_HIGH);
    collect_zero_range(PM_RANGE_HIGH, raw);
    assert(PM_GetZeroState() == PM_ZERO_RESTORING);
    service_to_range(PM_RANGE_MID);
    feed_raw(raw);
    assert(PM_GetZeroState() == PM_ZERO_DONE);
    assert(m->mode == PM_MODE_MANUAL && m->current_range == PM_RANGE_MID);
    const pm_power_parameters_t *after = PM_GetPowerParameters();
    near_value(after->zero_voltage[PM_RANGE_LOW], voltage);
    near_value(after->zero_voltage[PM_RANGE_MID], voltage);
    near_value(after->zero_voltage[PM_RANGE_HIGH], voltage);
    near_value(after->gain_correction[PM_RANGE_LOW], 1.3f);
    near_value(after->gain_correction[PM_RANGE_MID], 1.2f);
    near_value(after->gain_correction[PM_RANGE_HIGH], 1.1f);
    (void)before;
}

static void test_zero_restore_auto_and_failure_is_atomic(void)
{
    pm_test_reset(PM_RANGE_LOW);
    feed_raw(12000U);
    assert(PM_SetMode(PM_MODE_AUTO));
    assert(PM_StartZero());
    collect_zero_range(PM_RANGE_LOW, 12000U);
    service_to_range(PM_RANGE_MID);
    collect_zero_range(PM_RANGE_MID, 12000U);
    service_to_range(PM_RANGE_HIGH);
    collect_zero_range(PM_RANGE_HIGH, 12000U);
    assert(PM_GetZeroState() == PM_ZERO_DONE && m->mode == PM_MODE_AUTO);

    pm_test_reset(PM_RANGE_MID);
    feed_raw(20000U);
    pm_power_parameters_t before = *PM_GetPowerParameters();
    assert(PM_StartZero());
    pm_fake.pause_ok = false;
    assert(!PM_Service());
    PM_CalibrationService();
    assert(PM_GetZeroState() == PM_ZERO_FAILED);
    assert(memcmp(&before, PM_GetPowerParameters(), sizeof(before)) == 0);
}

static void test_calibration_and_parameter_validation(void)
{
    pm_test_reset(PM_RANGE_LOW);
    assert(PM_CheckCalibration(PM_RANGE_LOW, 0.0f) == PM_CAL_CHECK_REFERENCE_INVALID);
    feed_raw(20000U);
    assert(PM_CheckCalibration(PM_RANGE_MID, 5.0f) == PM_CAL_CHECK_RANGE_MISMATCH);
    assert(PM_CheckCalibration(PM_RANGE_LOW, 5.0f) == PM_CAL_CHECK_OK);
    assert(m->raw_power_valid && m->raw_power_mW > 0.0f);
    float reference = 5.0f;
    assert(PM_CalibrateRange(PM_RANGE_LOW, reference));
    near_value(PM_GetPowerParameters()->gain_correction[PM_RANGE_LOW], reference / m->raw_power_mW);
    near_value(m->power_mW, reference);
    assert(!PM_CalibrateRange(PM_RANGE_MID, reference));
    assert(!PM_CalibrateRange(PM_RANGE_LOW, 0.0f));
    assert(!PM_CalibrateRange(PM_RANGE_LOW, NAN));

    pm_power_parameters_t saved = *PM_GetPowerParameters();
    pm_power_parameters_t invalid = saved;
    invalid.gain_correction[PM_RANGE_HIGH] = 0.0f;
    assert(!PM_SetPowerParameters(&invalid));
    invalid = saved;
    invalid.zero_voltage[PM_RANGE_MID] = NAN;
    assert(!PM_SetPowerParameters(&invalid));
    assert(memcmp(&saved, PM_GetPowerParameters(), sizeof(saved)) == 0);

    assert(PM_SetRangeCorrection(PM_RANGE_LOW, m->filtered_voltage_V, 1.0f));
    assert(PM_CheckCalibration(PM_RANGE_LOW, 5.0f) == PM_CAL_CHECK_SIGNAL_TOO_LOW);
    assert(PM_SetWavelength(400) == 400U);
    assert(PM_CheckCalibration(PM_RANGE_LOW, 5.0f) == PM_CAL_CHECK_RESPONSIVITY_INVALID);
}

int main(void)
{
    m = PM_GetMeasurement();
    test_zero_three_ranges_and_restore_manual();
    test_zero_restore_auto_and_failure_is_atomic();
    test_calibration_and_parameter_validation();
    puts("PASS: ZERO LOW/MID/HIGH, valid-block averaging, mode restoration, atomic failure and CAL checks.");
    return 0;
}
