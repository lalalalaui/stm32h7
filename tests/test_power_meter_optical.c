#include "power_meter_fake_port.h"
#include "power_meter_config.h"

#include <assert.h>
#include <fenv.h>
#include <float.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static const pm_measurement_t *m;

static void near_relative(float actual, float expected)
{
    assert(isfinite(actual));
    assert(fabsf(actual - expected) <= fmaxf(1e-12f, fabsf(expected) * 2e-6f));
}

static void feed_raw(uint16_t raw)
{
    uint16_t samples[] = {raw, raw};
    pm_fake.now_ms += 4U;
    assert(pm_test_submit(samples, 2U, pm_fake.range));
}

static void test_lut(void)
{
    near_relative(PM_GetResponsivity(660U), 0.45f);
    near_relative(PM_GetResponsivity(780U), 0.52f);
    near_relative(PM_GetResponsivity(830U), 0.54f);
    near_relative(PM_GetResponsivity(960U), 0.60f);
    near_relative(PM_GetResponsivity(720U), 0.4825f);
    near_relative(PM_GetResponsivity(805U), 0.530f);
    near_relative(PM_GetResponsivity(895U), 0.56785715f);
    assert(PM_GetResponsivity(0U) == 0.0f);
    assert(PM_GetResponsivity(UINT16_MAX) == 0.0f);
    for (uint16_t nm = 400U; nm <= 1000U; nm++) {
        assert(PM_SetWavelength(nm) == nm);
        float response = PM_GetResponsivity(nm);
        assert(response > 0.0f && response <= 0.60f);
    }
    assert(PM_SetWavelength(INT32_MIN) == 400U);
    assert(PM_SetWavelength(INT32_MAX) == 1000U);
}

static void test_nominal_and_independent_corrections(void)
{
    const float resistance[] = {22000.0f, 1.0f / (1.0f / 22000.0f + 1.0f / 2210.0f),
                               1.0f / (1.0f / 22000.0f + 1.0f / 180.0f)};
    const pm_power_parameters_t *parameters = PM_GetPowerParameters();
    pm_test_reset(PM_RANGE_LOW);
    for (unsigned i = 0U; i < 3U; i++) {
        assert(parameters->zero_voltage[i] == 0.0f);
        assert(parameters->gain_correction[i] == 1.0f);
    }
    for (unsigned i = 0U; i < 3U; i++) {
        pm_range_t range = (pm_range_t)i;
        pm_test_manual_switch(range);
        assert(!m->power_valid && !m->dbm_valid);
        feed_raw(20000U);
        float voltage = 20000.0f / 65535.0f * 3.3f;
        float current = voltage / resistance[i];
        float power = current / 0.45f;
        assert(m->valid && m->power_valid && m->dbm_valid);
        near_relative(m->signal_voltage_V, voltage);
        near_relative(m->photocurrent_A, current);
        near_relative(m->power_W, power);
        near_relative(m->power_mW, power * 1000.0f);
        near_relative(m->power_uW, power * 1000000.0f);
        near_relative(m->power_dBm, 10.0f * log10f(power * 1000.0f));
        assert(PM_SetRangeCorrection(range, 0.1f * (float)(i + 1U), 1.1f + (float)i));
        current = (voltage - parameters->zero_voltage[i]) / resistance[i];
        near_relative(m->photocurrent_A, current);
        near_relative(m->power_W, current / 0.45f * parameters->gain_correction[i]);
        for (unsigned j = i + 1U; j < 3U; j++) {
            assert(parameters->zero_voltage[j] == 0.0f && parameters->gain_correction[j] == 1.0f);
        }
    }
    for (unsigned i = 0U; i < 3U; i++) {
        pm_test_manual_switch((pm_range_t)i);
        feed_raw(20000U);
        float current = (m->filtered_voltage_V - parameters->zero_voltage[i]) / resistance[i];
        near_relative(m->power_W, current / 0.45f * parameters->gain_correction[i]);
    }
}

static void test_invalid_and_zero(void)
{
    pm_test_reset(PM_RANGE_LOW);
    feclearexcept(FE_ALL_EXCEPT);
    feed_raw(0U);
    assert(m->power_valid && m->out_of_spec && !m->dbm_valid);
    assert(m->power_W == 0.0f && m->power_mW == 0.0f && m->power_uW == 0.0f);
    assert(m->power_dBm == PM_DBM_INVALID_VALUE && isfinite(m->power_dBm));
    assert((fetestexcept(FE_DIVBYZERO | FE_INVALID)) == 0);
    feed_raw(20000U);
    assert(PM_SetRangeCorrection(PM_RANGE_LOW, 2.0f, 1.0f));
    assert(m->signal_voltage_V == 0.0f && m->photocurrent_A == 0.0f);
    assert(m->power_valid && !m->dbm_valid && m->power_W == 0.0f);

    pm_measurement_t saved = *m;
    pm_power_parameters_t saved_parameters = *PM_GetPowerParameters();
    assert(!PM_SetRangeCorrection((pm_range_t)-1, 0.0f, 1.0f));
    assert(!PM_SetRangeCorrection((pm_range_t)3, 0.0f, 1.0f));
    assert(!PM_SetRangeCorrection(PM_RANGE_LOW, NAN, 1.0f));
    assert(!PM_SetRangeCorrection(PM_RANGE_LOW, INFINITY, 1.0f));
    assert(!PM_SetRangeCorrection(PM_RANGE_LOW, 0.0f, NAN));
    assert(!PM_SetRangeCorrection(PM_RANGE_LOW, 0.0f, INFINITY));
    assert(!PM_SetRangeCorrection(PM_RANGE_LOW, 0.0f, 0.0f));
    assert(!PM_SetRangeCorrection(PM_RANGE_LOW, 0.0f, -1.0f));
    assert(memcmp(&saved, m, sizeof(saved)) == 0);
    assert(memcmp(&saved_parameters, PM_GetPowerParameters(), sizeof(saved_parameters)) == 0);
    assert(PM_SetRangeCorrection(PM_RANGE_LOW, 0.0f, FLT_MAX));
    assert(m->valid && !m->power_valid && !m->dbm_valid);
    assert(isfinite(m->power_W) && isfinite(m->power_uW));
    assert(PM_SetRangeCorrection(PM_RANGE_LOW, 0.0f, 1.0f));
    assert(m->power_valid);
}

static void test_recomputation_and_validity(void)
{
    pm_test_reset(PM_RANGE_LOW);
    feed_raw(20000U);
    float old_power = m->power_W, old_filtered = m->filtered_voltage_V;
    uint32_t old_blocks = m->processed_blocks, old_id = pm_fake.acquisition_id;
    assert(PM_SetWavelength(780) == 780U);
    near_relative(m->power_W, old_power * 0.45f / 0.52f);
    assert(m->filtered_voltage_V == old_filtered && m->processed_blocks == old_blocks);
    assert(pm_fake.acquisition_id == old_id && pm_fake.pauses == 0U && pm_fake.switches == 0U);
    assert(PM_SetWavelength(650) == 650U);
    assert(m->valid && m->power_valid && m->responsivity_A_W > 0.0f);
    near_relative(m->power_W, old_power * 0.45f / PM_GetResponsivity(650U));
    assert(PM_SetWavelength(660) == 660U);
    near_relative(m->power_W, old_power);

    assert(PM_SetManualRange(PM_RANGE_MID));
    assert(!PM_Service());
    assert(!m->valid && !m->power_valid && m->power_W == 0.0f);
    (void)PM_SetWavelength(830);
    assert(!m->power_valid);
    pm_fake.now_ms += 3U;
    assert(PM_Service());
    uint16_t old_samples[] = {20000U};
    pm_sample_info_t old_info = {.completed_ms = pm_fake.now_ms, .sequence = ++pm_fake.sequence,
        .acquisition_id = old_id, .range = PM_RANGE_MID};
    assert(!PM_ProcessADCSamples(old_samples, 1U, &old_info));
    assert(!m->power_valid);
    feed_raw(10000U);
    assert(m->power_valid);
    near_relative(m->filtered_voltage_V, m->adc_voltage_V);
    near_relative(m->power_W, m->adc_voltage_V / PM_TIA_MID_OHM / 0.54f);
    assert(PM_SetVDDA(3.0f));
    assert(!m->valid && !m->power_valid && m->power_W == 0.0f);
    feed_raw(10000U);
    assert(m->power_valid);
    pm_fake.running = false;
    assert(!PM_Service());
    assert(m->adc_fault && !m->power_valid && !m->dbm_valid);
}

static void test_spec_and_overload(void)
{
    const float targets_mW[] = {0.099f, 0.101f, 1.0f, 19.999f, 20.001f};
    pm_test_reset(PM_RANGE_LOW);
    assert(PM_SetMode(PM_MODE_MANUAL));
    feed_raw(20000U);
    float nominal_mW = m->power_mW;
    for (unsigned i = 0U; i < sizeof(targets_mW) / sizeof(targets_mW[0]); i++) {
        assert(PM_SetRangeCorrection(PM_RANGE_LOW, 0.0f, targets_mW[i] / nominal_mW));
        assert(m->power_valid);
        near_relative(m->power_mW, targets_mW[i]);
        assert(m->under_range == (targets_mW[i] < 0.1f));
        assert(m->out_of_spec == (targets_mW[i] < 0.1f || targets_mW[i] > 20.0f));
        assert(m->valid && pm_fake.running);
    }
    feed_raw(62000U);
    assert(m->valid && m->over_range && !m->power_valid);
    feed_raw(50000U);
    assert(m->valid && !m->over_range && m->power_valid);
    pm_test_manual_switch(PM_RANGE_HIGH);
    feed_raw(65535U);
    assert(m->valid && !m->power_valid); /* Manual high-gain ADC clipping. */
}

int main(void)
{
    m = PM_GetMeasurement();
    assert(PM_SetWavelength(780) == 0U);
    assert(!PM_SetRangeCorrection(PM_RANGE_LOW, 0.0f, 1.0f));
    pm_test_reset(PM_RANGE_LOW);
    assert(m->wavelength_nm == 660U && !m->power_valid);
    test_lut();
    test_nominal_and_independent_corrections();
    test_invalid_and_zero();
    test_recomputation_and_validity();
    test_spec_and_overload();
    puts("PASS: LUT/interpolation, all 601 wavelengths, clamps, three gains/zeros, units/dBm, zero, invalid input, switch fencing, VDDA, spec and overload.");
    return 0;
}
