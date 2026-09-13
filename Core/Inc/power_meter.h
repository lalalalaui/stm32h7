#ifndef POWER_METER_H
#define POWER_METER_H

#include <stdbool.h>
#include <stdint.h>
#include "power_meter_hw.h"
#include "responsivity_table.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    PM_MODE_AUTO = 0,
    PM_MODE_MANUAL
} pm_mode_t;

typedef struct
{
    uint32_t completed_ms;
    uint32_t sequence;
    uint32_t acquisition_id; /* Changed every time DMA is restarted. */
    pm_range_t range;
} pm_sample_info_t;

/* Indexed by PM_RANGE_HIGH / MID / LOW. RAM only, reset by PM_Init(). */
typedef struct
{
    float zero_voltage[3];       /* Volts, initially 0. */
    float gain_correction[3];    /* Dimensionless, initially 1. */
} pm_power_parameters_t;

typedef enum
{
    PM_ZERO_IDLE = 0,
    PM_ZERO_LOW,
    PM_ZERO_MID,
    PM_ZERO_HIGH,
    PM_ZERO_RESTORING,
    PM_ZERO_DONE,
    PM_ZERO_FAILED
} pm_zero_state_t;

typedef enum
{
    PM_CAL_CHECK_OK = 0,
    PM_CAL_CHECK_REFERENCE_INVALID,
    PM_CAL_CHECK_ADC_INVALID,
    PM_CAL_CHECK_RANGE_SWITCHING,
    PM_CAL_CHECK_RANGE_MISMATCH,
    PM_CAL_CHECK_OVER_RANGE,
    PM_CAL_CHECK_RESPONSIVITY_INVALID,
    PM_CAL_CHECK_SIGNAL_TOO_LOW,
    PM_CAL_CHECK_RAW_POWER_INVALID
} pm_calibration_check_t;

typedef struct
{
    uint16_t adc_raw;           /* Last individual sample in the latest block. */
    float adc_mean;            /* Block average in ADC counts, fractional. */
    uint16_t adc_min;          /* Latest block minimum, ADC counts. */
    uint16_t adc_max;          /* Latest block maximum, ADC counts. */
    float adc_voltage_V;       /* Block average converted using VDDA. */
    float filtered_voltage_V;  /* IIR applied once per consumed block. */
    float vdda_V;
    pm_range_t current_range;  /* Commanded range when the block is submitted. */
    uint16_t sample_count;
    uint32_t processed_blocks;
    bool valid;               /* False until a valid sample block is processed. */
    pm_mode_t mode;
    pm_range_t target_range;
    bool range_switching;     /* Includes settling and waiting for a NEW full block. */
    bool over_range;
    bool adc_fault;           /* Pause/resume or GPIO operation failed; restart required. */
    uint16_t wavelength_nm;
    float responsivity_A_W;   /* 0 means wavelength has no reliable LUT coverage. */
    float wavelength_correction;
    bool wavelength_calibrated;
    float signal_voltage_V;
    float photocurrent_A;
    float raw_power_W;       /* Before per-range gain correction. */
    float raw_power_mW;
    float power_W;
    float power_uW;
    float power_mW;
    float power_dBm;
    bool power_valid;         /* Separate from ADC validity; false on missing LUT/overload. */
    bool raw_power_valid;
    bool dbm_valid;           /* False at zero power or when power_valid is false. */
    bool under_range;         /* Valid calculated power below the 0.1 mW target minimum. */
    bool out_of_spec;         /* Valid calculated power outside 0.1..20 mW. */
} pm_measurement_t;

/* Main-loop-only API. Hardware operations go through power_meter_port.c. */
void PM_Init(pm_range_t initial_range);
/* Call before reading a DMA snapshot; false means do not consume any samples. */
bool PM_Service(void);
bool PM_ProcessADCSamples(const uint16_t *samples, uint16_t count, const pm_sample_info_t *info);
const pm_measurement_t *PM_GetMeasurement(void);
bool PM_SetMode(pm_mode_t mode);
/* Queues a safe transition and selects MANUAL; PM_Service performs it. */
bool PM_SetManualRange(pm_range_t range);

/* Runtime voltage-conversion configuration for future measured VDDA input.
 * Rejects nonpositive/nonfinite values and VDDA above PM_VDDA_MAX_V.
 * Does not calibrate or save parameters.
 * On change, invalidates old voltages until the next block reseeds the IIR.
 */
bool PM_SetVDDA(float vdda_V);

/* Integer-nm setting, clamps even signed/out-of-range inputs to 400..1000.
 * Recomputes power immediately without resetting ADC/IIR or auto-range timing.
 * Returns actual wavelength (0 if PM_Init has not yet been called).
 */
uint16_t PM_SetWavelength(int32_t wavelength_nm);
const pm_power_parameters_t *PM_GetPowerParameters(void);
/* Configuration hook, not a calibration procedure or persistent storage.
 * Accepts finite zero offset (V) and finite strictly-positive gain multiplier.
 * Invalid arguments leave all parameters and measurements unchanged.
 */
bool PM_SetRangeCorrection(pm_range_t range, float zero_voltage_V, float gain_correction);

/* Atomic parameter replacement for EEPROM restore/rollback. */
bool PM_SetPowerParameters(const pm_power_parameters_t *parameters);

/* Blocking-light ZERO workflow. It samples LOW, MID, then HIGH while keeping
 * candidate values private. Parameters change only after all three pass.
 * Call PM_CalibrationService() from the main loop as well as processing ADC
 * blocks through PM_ProcessADCSamples().
 */
bool PM_StartZero(void);
void PM_CalibrationService(void);
pm_zero_state_t PM_GetZeroState(void);

/* Validates all prerequisites without changing any calibration parameter.
 * The UI uses this result to tell the operator exactly why CAL is blocked. */
pm_calibration_check_t PM_CheckCalibration(pm_range_t range, float reference_mW);

/* Reference power is in mW. Current range must equal range and be valid.
 * Replaces only this range's correction: reference_mW/raw_power_mW.
 */
bool PM_CalibrateRange(pm_range_t range, float reference_mW);
bool PM_CalibrateWavelength(float reference_mW);

#ifdef __cplusplus
}
#endif

#endif /* POWER_METER_H */
