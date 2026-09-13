#ifndef RESPONSIVITY_TABLE_H
#define RESPONSIVITY_TABLE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PM_WAVELENGTH_MIN_NM 400U
#define PM_WAVELENGTH_MAX_NM 1000U
#define PM_WAVELENGTH_CAL_MAX_POINTS 5U

typedef struct { uint16_t wavelength_nm; float correction_factor; } pm_wavelength_cal_point_t;

/* Typical S1223-01 responsivity in A/W, piecewise linear within the LUT.
 * Returns a typical, non-calibrated value throughout 400..1000 nm.
 */
float PM_GetResponsivity(uint16_t wavelength_nm);
float PM_GetWavelengthCorrection(uint16_t wavelength_nm, bool *has_exact_point);
uint8_t PM_GetWavelengthCalibrationPoints(pm_wavelength_cal_point_t *points, uint8_t capacity);
bool PM_SetWavelengthCalibrationPoint(uint16_t wavelength_nm, float correction_factor);
bool PM_DeleteWavelengthCalibrationPoint(uint16_t wavelength_nm);
void PM_ResetWavelengthCalibration(void);

#ifdef __cplusplus
}
#endif

#endif /* RESPONSIVITY_TABLE_H */
