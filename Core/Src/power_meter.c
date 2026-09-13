#include "power_meter.h"
#include "power_meter_config.h"
#include "power_meter_port.h"

#include <math.h>
#include <stddef.h>

static pm_measurement_t g_measurement;
static pm_power_parameters_t g_power_parameters;
static const float g_transimpedance_ohm[3] =
{
    [PM_RANGE_HIGH] = PM_TIA_HIGH_OHM,
    [PM_RANGE_MID] = PM_TIA_MID_OHM,
    [PM_RANGE_LOW] = PM_TIA_LOW_OHM
};
typedef enum { PM_PHASE_MEASURING, PM_PHASE_SETTLING, PM_PHASE_FAULT } pm_phase_t;
static pm_phase_t g_phase;
static bool g_initialized;
static uint32_t g_range_generation;
static uint32_t g_acquisition_id;
static uint32_t g_settle_start_ms;
static bool g_have_frame;
static uint32_t g_last_sequence;
static uint32_t g_last_frame_ms;
static bool g_up_pending;
static bool g_down_pending;
static uint32_t g_up_start_ms;
static uint32_t g_down_start_ms;
typedef struct
{
    pm_zero_state_t state;
    pm_mode_t saved_mode;
    pm_range_t saved_target_range;
    pm_power_parameters_t candidate;
    float sum_voltage_V;
    uint8_t block_count;
    bool discard_first_block;
} pm_zero_context_t;
static pm_zero_context_t g_zero;

static void pm_update_power(void)
{
    g_measurement.responsivity_A_W = PM_GetResponsivity(g_measurement.wavelength_nm);
    g_measurement.wavelength_correction = PM_GetWavelengthCorrection(g_measurement.wavelength_nm,
                                                                       &g_measurement.wavelength_calibrated);
    g_measurement.signal_voltage_V = 0.0f;
    g_measurement.photocurrent_A = 0.0f;
    g_measurement.raw_power_W = 0.0f;
    g_measurement.raw_power_mW = 0.0f;
    g_measurement.power_W = 0.0f;
    g_measurement.power_uW = 0.0f;
    g_measurement.power_mW = 0.0f;
    g_measurement.power_dBm = PM_DBM_INVALID_VALUE;
    g_measurement.power_valid = false;
    g_measurement.raw_power_valid = false;
    g_measurement.dbm_valid = false;
    g_measurement.under_range = false;
    g_measurement.out_of_spec = false;
    if (!g_measurement.valid) { return; }

    pm_range_t range = g_measurement.current_range;
    float signal = g_measurement.filtered_voltage_V - g_power_parameters.zero_voltage[range];
    if (!isfinite(signal)) { return; }
    if (signal < 0.0f) { signal = 0.0f; }
    g_measurement.signal_voltage_V = signal;
    g_measurement.photocurrent_A = signal / g_transimpedance_ohm[range];
    if (g_measurement.responsivity_A_W <= 0.0f || g_measurement.over_range ||
        g_measurement.adc_mean >= (float)PM_ADC_FULL_SCALE)
    {
        /* Keep ADC diagnostics, but do not publish clipped ADC data as optical power. */
        return;
    }

    float raw_power_W = g_measurement.photocurrent_A / g_measurement.responsivity_A_W;
    float raw_power_mW = raw_power_W * 1000.0f;
    if (!isfinite(raw_power_W) || !isfinite(raw_power_mW)) { return; }
    g_measurement.raw_power_W = raw_power_W;
    g_measurement.raw_power_mW = raw_power_mW;
    g_measurement.raw_power_valid = true;
    float power_W = raw_power_W * g_power_parameters.gain_correction[range] *
                    g_measurement.wavelength_correction;
    float power_mW = power_W * 1000.0f;
    float power_uW = power_W * 1000000.0f;
    if (!isfinite(power_W) || !isfinite(power_mW) || !isfinite(power_uW)) { return; }
    g_measurement.power_W = power_W;
    g_measurement.power_mW = power_mW;
    g_measurement.power_uW = power_uW;
    g_measurement.power_valid = true;
    g_measurement.under_range = power_mW < PM_POWER_SPEC_MIN_MW;
    g_measurement.out_of_spec = g_measurement.under_range || power_mW > PM_POWER_SPEC_MAX_MW;
    if (power_mW > 0.0f)
    {
        g_measurement.power_dBm = 10.0f * log10f(power_mW);
        g_measurement.dbm_valid = true;
    }
}

static void pm_reset_confirmation(void)
{
    g_up_pending = false;
    g_down_pending = false;
}

static bool pm_fault(void)
{
    g_measurement.valid = false;
    g_measurement.range_switching = false;
    g_measurement.over_range = false;
    g_measurement.adc_fault = true;
    pm_update_power();
    g_phase = PM_PHASE_FAULT;
    pm_reset_confirmation();
    return false;
}

static bool pm_range_valid(pm_range_t range)
{
    return range == PM_RANGE_HIGH || range == PM_RANGE_MID || range == PM_RANGE_LOW;
}

static pm_range_t pm_zero_range_for_state(pm_zero_state_t state)
{
    if (state == PM_ZERO_LOW) { return PM_RANGE_LOW; }
    if (state == PM_ZERO_MID) { return PM_RANGE_MID; }
    return PM_RANGE_HIGH;
}

static pm_zero_state_t pm_zero_next_state(pm_zero_state_t state)
{
    if (state == PM_ZERO_LOW) { return PM_ZERO_MID; }
    if (state == PM_ZERO_MID) { return PM_ZERO_HIGH; }
    return PM_ZERO_RESTORING;
}

static void pm_zero_fail(void)
{
    if (g_zero.state != PM_ZERO_IDLE && g_zero.state != PM_ZERO_DONE)
    {
        g_zero.state = PM_ZERO_FAILED;
    }
}

static void pm_zero_commit(void)
{
    /* Candidate offsets stay private until every range and the restore path
     * have succeeded, so a failed ZERO never changes an existing offset. */
    g_power_parameters = g_zero.candidate;
    pm_update_power();
    g_zero.state = PM_ZERO_DONE;
}

static void pm_zero_process_block(void)
{
    if (g_zero.state == PM_ZERO_IDLE || g_zero.state == PM_ZERO_DONE ||
        g_zero.state == PM_ZERO_FAILED) { return; }
    if (g_measurement.adc_fault) { pm_zero_fail(); return; }

    if (g_zero.state == PM_ZERO_RESTORING)
    {
        if (g_measurement.valid && !g_measurement.range_switching &&
            g_measurement.current_range == g_zero.saved_target_range)
        {
            pm_zero_commit();
        }
        return;
    }

    pm_range_t target_range = pm_zero_range_for_state(g_zero.state);
    if (!g_measurement.valid || g_measurement.range_switching ||
        g_measurement.current_range != target_range)
    {
        return;
    }
    if (g_zero.discard_first_block)
    {
        g_zero.discard_first_block = false;
        return;
    }

    g_zero.sum_voltage_V += g_measurement.adc_voltage_V;
    g_zero.block_count++;
    if (g_zero.block_count < PM_ZERO_BLOCKS_PER_RANGE) { return; }

    g_zero.candidate.zero_voltage[target_range] =
        g_zero.sum_voltage_V / (float)PM_ZERO_BLOCKS_PER_RANGE;
    g_zero.sum_voltage_V = 0.0f;
    g_zero.block_count = 0U;
    if (g_zero.state != PM_ZERO_HIGH)
    {
        g_zero.state = pm_zero_next_state(g_zero.state);
        g_zero.discard_first_block = true;
        if (!PM_SetManualRange(pm_zero_range_for_state(g_zero.state))) { pm_zero_fail(); }
        return;
    }

    /* All three candidates are valid. Keep them private until the original
     * mode/range is restored as well. */
    if (g_zero.saved_mode == PM_MODE_AUTO)
    {
        if (!PM_SetMode(PM_MODE_AUTO)) { pm_zero_fail(); }
        else { pm_zero_commit(); }
    }
    else
    {
        g_zero.state = PM_ZERO_RESTORING;
        g_zero.discard_first_block = true;
        if (!PM_SetManualRange(g_zero.saved_target_range)) { pm_zero_fail(); }
    }
}

void PM_Init(pm_range_t initial_range)
{
    g_measurement = (pm_measurement_t){0};
    g_measurement.vdda_V = PM_VDDA_DEFAULT_V;
    g_measurement.current_range = pm_range_valid(initial_range) ? initial_range : PM_RANGE_LOW;
    g_measurement.target_range = g_measurement.current_range;
    g_measurement.mode = PM_MODE_AUTO;
    g_measurement.wavelength_nm = PM_WAVELENGTH_DEFAULT_NM;
    g_power_parameters = (pm_power_parameters_t){
        .zero_voltage = {0.0f, 0.0f, 0.0f},
        .gain_correction = {1.0f, 1.0f, 1.0f}
    };
    g_zero = (pm_zero_context_t){.state = PM_ZERO_IDLE};
    pm_update_power();
    g_phase = PM_PHASE_MEASURING;
    g_initialized = true;
    g_range_generation = PM_PortRangeGeneration();
    g_acquisition_id = PM_PortAcquisitionId();
    g_have_frame = false;
    pm_reset_confirmation();
}

bool PM_SetMode(pm_mode_t mode)
{
    if (!g_initialized || g_measurement.adc_fault ||
        (mode != PM_MODE_AUTO && mode != PM_MODE_MANUAL)) { return false; }
    g_measurement.mode = mode;
    g_measurement.target_range = g_measurement.current_range;
    pm_reset_confirmation();
    return true;
}

bool PM_SetManualRange(pm_range_t range)
{
    if (!g_initialized || !pm_range_valid(range) || !PM_SetMode(PM_MODE_MANUAL)) { return false; }
    g_measurement.target_range = range;
    return true;
}

bool PM_Service(void)
{
    if (!g_initialized || g_phase == PM_PHASE_FAULT) { pm_zero_fail(); return false; }

    pm_range_t actual_range = PM_PortGetRange();
    bool external_change = PM_PortRangeGeneration() != g_range_generation;
    if (external_change)
    {
        /* Legacy direct HW calls are detected even for HIGH->LOW->HIGH.
         * Discard the entire acquisition and fence it before publishing data.
         */
        g_measurement.mode = PM_MODE_MANUAL;
        g_measurement.target_range = actual_range;
    }

    if (external_change || g_measurement.target_range != actual_range)
    {
        g_measurement.valid = false;
        g_measurement.range_switching = true;
        g_measurement.over_range = false;
        pm_update_power();
        g_have_frame = false;
        pm_reset_confirmation();
        if (!PM_PortPauseADC()) { return pm_fault(); }
        if (!external_change && !PM_PortSetRange(g_measurement.target_range)) { return pm_fault(); }
        g_measurement.current_range = PM_PortGetRange();
        if (!pm_range_valid(g_measurement.current_range)) { return pm_fault(); }
        g_range_generation = PM_PortRangeGeneration();
        g_settle_start_ms = PM_PortNowMs(); /* Timestamp AFTER GPIO sequencing. */
        g_phase = PM_PHASE_SETTLING;
        return false;
    }

    if (g_phase == PM_PHASE_SETTLING)
    {
        /* +1 tick ensures >=2 ms physical time despite millisecond quantization. */
        if ((uint32_t)(PM_PortNowMs() - g_settle_start_ms) < PM_RANGE_SETTLE_MS + 1U) { return false; }
        if (!PM_PortResumeADC()) { return pm_fault(); }
        g_acquisition_id = PM_PortAcquisitionId();
        g_phase = PM_PHASE_MEASURING;
        /* valid stays false until a full new-epoch block is submitted. */
    }

    if (!PM_PortADCIsRunning()) { return pm_fault(); }
    if (g_acquisition_id != PM_PortAcquisitionId())
    {
        /* Also handle the initial ADC start, which follows PM_Init(). */
        g_acquisition_id = PM_PortAcquisitionId();
        g_measurement.valid = false;
        pm_update_power();
        g_have_frame = false;
        pm_reset_confirmation();
    }
    return true;
}

static void pm_update_auto_range(uint32_t completed_ms)
{
    float voltage_V = g_measurement.adc_voltage_V; /* NOT the slow display IIR. */
    pm_range_t range = g_measurement.current_range;
    if (range == PM_RANGE_LOW)
    {
        if (voltage_V > PM_OVER_RANGE_V) { g_measurement.over_range = true; }
        else if (voltage_V < PM_OVER_RANGE_CLEAR_V) { g_measurement.over_range = false; }
    }
    else { g_measurement.over_range = false; }

    if (g_measurement.mode != PM_MODE_AUTO) { pm_reset_confirmation(); return; }
    if (voltage_V > PM_RANGE_DOWN_V && range != PM_RANGE_LOW)
    {
        g_up_pending = false;
        if (!g_down_pending) { g_down_pending = true; g_down_start_ms = completed_ms; }
        if ((uint32_t)(completed_ms - g_down_start_ms) >= PM_RANGE_DOWN_HOLD_MS)
        {
            g_measurement.target_range = range == PM_RANGE_HIGH ? PM_RANGE_MID : PM_RANGE_LOW;
            pm_reset_confirmation();
        }
    }
    else if (voltage_V < PM_RANGE_UP_V && range != PM_RANGE_HIGH)
    {
        g_down_pending = false;
        if (!g_up_pending) { g_up_pending = true; g_up_start_ms = completed_ms; }
        if ((uint32_t)(completed_ms - g_up_start_ms) >= PM_RANGE_UP_HOLD_MS)
        {
            g_measurement.target_range = range == PM_RANGE_LOW ? PM_RANGE_MID : PM_RANGE_HIGH;
            pm_reset_confirmation();
        }
    }
    else { pm_reset_confirmation(); }
}

bool PM_ProcessADCSamples(const uint16_t *samples, uint16_t count, const pm_sample_info_t *info)
{
    uint32_t sum = 0U;
    uint16_t minimum = (uint16_t)PM_ADC_FULL_SCALE;
    uint16_t maximum = 0U;
    float mean;
    float voltage_V;
    float filtered_V;

    if (!g_initialized || samples == NULL || count == 0U || info == NULL ||
        g_phase != PM_PHASE_MEASURING || info->range != g_measurement.current_range ||
        info->acquisition_id != g_acquisition_id ||
        PM_PortRangeGeneration() != g_range_generation ||
        !isfinite(g_measurement.vdda_V) || g_measurement.vdda_V <= 0.0f)
    {
        return false;
    }

    if ((uint32_t)(PM_PortNowMs() - info->completed_ms) > PM_SAMPLE_MAX_GAP_MS)
    {
        pm_reset_confirmation();
        return false;
    }
    if (g_have_frame)
    {
        uint32_t advance = info->sequence - g_last_sequence;
        if (advance == 0U || advance >= 0x80000000UL) { return false; }
        if (advance != 1U || (uint32_t)(info->completed_ms - g_last_frame_ms) > PM_SAMPLE_MAX_GAP_MS)
        {
            pm_reset_confirmation();
        }
    }

    /* uint16_t count and samples bound the sum to 65535 * 65535, fitting u32. */
    for (uint16_t i = 0U; i < count; i++)
    {
        uint16_t raw = samples[i];
#if PM_ADC_RESOLUTION_BITS < 16U
        if (raw > PM_ADC_FULL_SCALE)
        {
            return false;
        }
#endif
        sum += raw;
        if (raw < minimum) { minimum = raw; }
        if (raw > maximum) { maximum = raw; }
    }

    mean = (float)sum / (float)count;
    voltage_V = (mean / (float)PM_ADC_FULL_SCALE) * g_measurement.vdda_V;

    if (!g_measurement.valid)
    {
        filtered_V = voltage_V;
    }
    else
    {
        filtered_V = PM_IIR_ALPHA * voltage_V +
                     (1.0f - PM_IIR_ALPHA) * g_measurement.filtered_voltage_V;
    }

    g_measurement.adc_raw = samples[count - 1U];
    g_measurement.adc_mean = mean;
    g_measurement.adc_min = minimum;
    g_measurement.adc_max = maximum;
    g_measurement.adc_voltage_V = voltage_V;
    g_measurement.filtered_voltage_V = filtered_V;
    g_measurement.sample_count = count;
    g_measurement.processed_blocks++;
    g_measurement.valid = true;
    g_measurement.range_switching = false;
    g_have_frame = true;
    g_last_sequence = info->sequence;
    g_last_frame_ms = info->completed_ms;
    pm_update_auto_range(info->completed_ms);
    pm_update_power();
    pm_zero_process_block();
    return true;
}

const pm_measurement_t *PM_GetMeasurement(void)
{
    return &g_measurement;
}

bool PM_SetVDDA(float vdda_V)
{
    if (!isfinite(vdda_V) || vdda_V <= 0.0f || vdda_V > PM_VDDA_MAX_V)
    {
        return false;
    }
    if (g_measurement.vdda_V != vdda_V)
    {
        g_measurement.vdda_V = vdda_V;
        g_measurement.valid = false;
        pm_update_power();
        pm_reset_confirmation();
    }
    return true;
}

uint16_t PM_SetWavelength(int32_t wavelength_nm)
{
    if (!g_initialized) { return 0U; }
    if (wavelength_nm < (int32_t)PM_WAVELENGTH_MIN_NM) { wavelength_nm = PM_WAVELENGTH_MIN_NM; }
    if (wavelength_nm > (int32_t)PM_WAVELENGTH_MAX_NM) { wavelength_nm = PM_WAVELENGTH_MAX_NM; }
    g_measurement.wavelength_nm = (uint16_t)wavelength_nm;
    pm_update_power();
    return g_measurement.wavelength_nm;
}

const pm_power_parameters_t *PM_GetPowerParameters(void)
{
    return &g_power_parameters;
}

bool PM_SetRangeCorrection(pm_range_t range, float zero_voltage_V, float gain_correction)
{
    if (!g_initialized || !pm_range_valid(range) || !isfinite(zero_voltage_V) ||
        !isfinite(gain_correction) || gain_correction <= 0.0f)
    {
        return false;
    }
    g_power_parameters.zero_voltage[range] = zero_voltage_V;
    g_power_parameters.gain_correction[range] = gain_correction;
    pm_update_power();
    return true;
}

bool PM_SetPowerParameters(const pm_power_parameters_t *parameters)
{
    if (!g_initialized || parameters == NULL) { return false; }
    for (uint32_t i = 0U; i < 3U; i++)
    {
        if (!isfinite(parameters->zero_voltage[i]) ||
            !isfinite(parameters->gain_correction[i]) || parameters->gain_correction[i] <= 0.0f)
        {
            return false;
        }
    }
    g_power_parameters = *parameters;
    pm_update_power();
    return true;
}

bool PM_StartZero(void)
{
    if (!g_initialized || g_measurement.adc_fault || !g_measurement.valid ||
        g_measurement.range_switching || g_zero.state == PM_ZERO_LOW ||
        g_zero.state == PM_ZERO_MID || g_zero.state == PM_ZERO_HIGH ||
        g_zero.state == PM_ZERO_RESTORING)
    {
        return false;
    }
    g_zero = (pm_zero_context_t){
        .state = PM_ZERO_LOW,
        .saved_mode = g_measurement.mode,
        .saved_target_range = g_measurement.target_range,
        .candidate = g_power_parameters,
        .discard_first_block = true
    };
    if (!PM_SetManualRange(PM_RANGE_LOW))
    {
        pm_zero_fail();
        return false;
    }
    return true;
}

void PM_CalibrationService(void)
{
    if (!g_initialized) { return; }
    if (g_measurement.adc_fault) { pm_zero_fail(); }
}

pm_zero_state_t PM_GetZeroState(void)
{
    return g_zero.state;
}

bool PM_CalibrateRange(pm_range_t range, float reference_mW)
{
    if (PM_CheckCalibration(range, reference_mW) != PM_CAL_CHECK_OK) { return false; }
    float correction = reference_mW / g_measurement.raw_power_mW;
    if (!isfinite(correction) || correction <= 0.0f) { return false; }
    return PM_SetRangeCorrection(range, g_power_parameters.zero_voltage[range], correction);
}

bool PM_CalibrateWavelength(float reference_mW)
{
    pm_range_t range=g_measurement.current_range;
    if(PM_CheckCalibration(range,reference_mW)!=PM_CAL_CHECK_OK)return false;
    float base_mW=g_measurement.raw_power_mW*g_power_parameters.gain_correction[range];
    float factor=reference_mW/base_mW;
    if(!isfinite(factor)||factor<=0.0f)return false;
    if(!PM_SetWavelengthCalibrationPoint(g_measurement.wavelength_nm,factor))return false;
    pm_update_power();
    return true;
}

pm_calibration_check_t PM_CheckCalibration(pm_range_t range, float reference_mW)
{
    if (!isfinite(reference_mW) || reference_mW <= 0.0f)
    {
        return PM_CAL_CHECK_REFERENCE_INVALID;
    }
    if (!g_initialized || !pm_range_valid(range) || g_measurement.adc_fault || !g_measurement.valid)
    {
        return PM_CAL_CHECK_ADC_INVALID;
    }
    if (g_measurement.range_switching)
    {
        return PM_CAL_CHECK_RANGE_SWITCHING;
    }
    if (g_measurement.current_range != range)
    {
        return PM_CAL_CHECK_RANGE_MISMATCH;
    }
    if (g_measurement.over_range)
    {
        return PM_CAL_CHECK_OVER_RANGE;
    }
    if (!isfinite(g_measurement.responsivity_A_W) || g_measurement.responsivity_A_W <= 0.0f)
    {
        return PM_CAL_CHECK_RESPONSIVITY_INVALID;
    }
    if (!isfinite(g_measurement.signal_voltage_V) ||
        g_measurement.signal_voltage_V < PM_CAL_MIN_SIGNAL_V)
    {
        return PM_CAL_CHECK_SIGNAL_TOO_LOW;
    }
    if (!g_measurement.raw_power_valid || !isfinite(g_measurement.raw_power_mW) ||
        g_measurement.raw_power_mW <= 0.0f)
    {
        return PM_CAL_CHECK_RAW_POWER_INVALID;
    }
    return PM_CAL_CHECK_OK;
}
