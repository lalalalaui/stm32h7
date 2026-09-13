#ifndef POWER_METER_CONFIG_H
#define POWER_METER_CONFIG_H

/* Conversion settings only: these do not reconfigure ADC/TIM6/DMA.
 * Must match the existing ADC BSP resolution (currently ADC_RESOLUTION_16B).
 */
#define PM_ADC_RESOLUTION_BITS  16U
#define PM_ADC_FULL_SCALE       ((1UL << PM_ADC_RESOLUTION_BITS) - 1UL)
#define PM_VDDA_DEFAULT_V       3.3f
#define PM_VDDA_MAX_V           3.6f
#define PM_IIR_ALPHA            0.25f

/* Nominal feedback network only; ADG721 Ron and actual component tolerances
 * are NOT included. Future per-range calibration supplies gain correction.
 */
#define PM_TIA_HIGH_OHM         22000.0f
#define PM_TIA_MID_OHM          (22000.0f * 2210.0f / (22000.0f + 2210.0f))
#define PM_TIA_LOW_OHM          (22000.0f * 180.0f / (22000.0f + 180.0f))
#define PM_WAVELENGTH_DEFAULT_NM 660U
#define PM_POWER_SPEC_MIN_MW    0.1f
#define PM_POWER_SPEC_MAX_MW    20.0f
/* Finite storage sentinel only. At zero power dbm_valid is false: show "--",
 * never interpret this sentinel as a measured dBm value or call log10f(0).
 */
#define PM_DBM_INVALID_VALUE    (-120.0f)

/* ZERO uses fresh, full DMA blocks after each already-fenced range change. */
#define PM_ZERO_BLOCKS_PER_RANGE 8U

/* A calibration point must remain sufficiently above its independently
 * measured zero offset. This avoids turning ADC noise into a large gain. */
#define PM_CAL_MIN_SIGNAL_V      0.005f

#define PM_RANGE_DOWN_V         2.40f
#define PM_RANGE_UP_V           0.15f
#define PM_RANGE_DOWN_HOLD_MS   6U
#define PM_RANGE_UP_HOLD_MS     80U
#define PM_RANGE_SETTLE_MS      2U
#define PM_OVER_RANGE_V         3.00f
#define PM_OVER_RANGE_CLEAR_V   2.90f
/* Discontinuous or stale snapshots must not count toward a hold interval. */
#define PM_SAMPLE_MAX_GAP_MS    20U

#if PM_ADC_RESOLUTION_BITS != 8U && PM_ADC_RESOLUTION_BITS != 10U && \
    PM_ADC_RESOLUTION_BITS != 12U && PM_ADC_RESOLUTION_BITS != 14U && \
    PM_ADC_RESOLUTION_BITS != 16U
#error "Unsupported power-meter ADC resolution"
#endif

#endif /* POWER_METER_CONFIG_H */
