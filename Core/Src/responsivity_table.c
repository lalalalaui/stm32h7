#include "responsivity_table.h"

#include <stddef.h>
#include <math.h>
#include <string.h>

typedef struct
{
    uint16_t wavelength_nm;
    float responsivity_A_W;
} pm_responsivity_point_t;

/* Hamamatsu S1223 series, KPIN1050E02, Feb. 2013, p.1:
 * Electrical and optical characteristics, S1223-01, typical at Ta=25 C.
 * https://www.hamamatsu.com/content/dam/hamamatsu-photonics/sites/documents/99_SALES_LIBRARY/ssd/s1223_series_kpin1050e.pdf
 * 960 nm is the tabulated typical peak wavelength (0.60 A/W at the peak).
 * 660/780/830/960 are tabulated official values. The remaining anchors are
 * approximate readings from the official typical spectral-response curve.
 * Wavelength calibration points correct a real instrument.
 */
static const pm_responsivity_point_t g_responsivity_table[] = {
    {400U,0.20f},{450U,0.28f},{500U,0.34f},{550U,0.38f},{600U,0.41f},{660U,0.45f},
    {700U,0.47f},{780U,0.52f},{830U,0.54f},{900U,0.57f},{960U,0.60f},{1000U,0.58f}
};
static pm_wavelength_cal_point_t g_cal_points[PM_WAVELENGTH_CAL_MAX_POINTS];
static uint8_t g_cal_count;

float PM_GetResponsivity(uint16_t wavelength_nm)
{
    const size_t count = sizeof(g_responsivity_table) / sizeof(g_responsivity_table[0]);
    if (wavelength_nm < PM_WAVELENGTH_MIN_NM || wavelength_nm > PM_WAVELENGTH_MAX_NM)
    {
        return 0.0f;
    }

    for (size_t i = 1U; i < count; i++)
    {
        const pm_responsivity_point_t *left = &g_responsivity_table[i - 1U];
        const pm_responsivity_point_t *right = &g_responsivity_table[i];
        if (wavelength_nm <= right->wavelength_nm)
        {
            float fraction = (float)(wavelength_nm - left->wavelength_nm) /
                             (float)(right->wavelength_nm - left->wavelength_nm);
            return left->responsivity_A_W +
                   fraction * (right->responsivity_A_W - left->responsivity_A_W);
        }
    }
    return 0.0f;
}

float PM_GetWavelengthCorrection(uint16_t wavelength_nm, bool *exact)
{
    if(exact!=NULL) { *exact=false; }
    if(g_cal_count==0U) { return 1.0f; }
    for(uint8_t i=0U;i<g_cal_count;i++)if(g_cal_points[i].wavelength_nm==wavelength_nm){if(exact!=NULL)*exact=true;return g_cal_points[i].correction_factor;}
    if(wavelength_nm<=g_cal_points[0].wavelength_nm)return g_cal_points[0].correction_factor;
    if(wavelength_nm>=g_cal_points[g_cal_count-1U].wavelength_nm)return g_cal_points[g_cal_count-1U].correction_factor;
    for(uint8_t i=1U;i<g_cal_count;i++)if(wavelength_nm<g_cal_points[i].wavelength_nm){
        pm_wavelength_cal_point_t *a=&g_cal_points[i-1U],*b=&g_cal_points[i]; float f=(float)(wavelength_nm-a->wavelength_nm)/(float)(b->wavelength_nm-a->wavelength_nm); return a->correction_factor+f*(b->correction_factor-a->correction_factor); }
    return 1.0f;
}
uint8_t PM_GetWavelengthCalibrationPoints(pm_wavelength_cal_point_t *points,uint8_t capacity)
{uint8_t n=g_cal_count<capacity?g_cal_count:capacity;if(points!=NULL&&n)memcpy(points,g_cal_points,n*sizeof(*points));return g_cal_count;}
bool PM_SetWavelengthCalibrationPoint(uint16_t wavelength_nm,float factor)
{if(wavelength_nm<400U||wavelength_nm>1000U||!isfinite(factor)||factor<=0.0f)return false;uint8_t i=0U;while(i<g_cal_count&&g_cal_points[i].wavelength_nm<wavelength_nm)i++;if(i<g_cal_count&&g_cal_points[i].wavelength_nm==wavelength_nm){g_cal_points[i].correction_factor=factor;return true;}if(g_cal_count>=PM_WAVELENGTH_CAL_MAX_POINTS)return false;memmove(&g_cal_points[i+1U],&g_cal_points[i],(g_cal_count-i)*sizeof(g_cal_points[0]));g_cal_points[i]=(pm_wavelength_cal_point_t){wavelength_nm,factor};g_cal_count++;return true;}
bool PM_DeleteWavelengthCalibrationPoint(uint16_t wavelength_nm)
{for(uint8_t i=0U;i<g_cal_count;i++)if(g_cal_points[i].wavelength_nm==wavelength_nm){memmove(&g_cal_points[i],&g_cal_points[i+1U],(g_cal_count-i-1U)*sizeof(g_cal_points[0]));g_cal_count--;return true;}return false;}
void PM_ResetWavelengthCalibration(void){g_cal_count=0U;memset(g_cal_points,0,sizeof(g_cal_points));}
