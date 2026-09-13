#include "power_meter_storage.h"
#include "power_meter.h"
#include "./BSP/24CXX/24cxx.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define PM_STORAGE_MAGIC   0x504D5436UL /* "PMT6" */
#define PM_STORAGE_VERSION 2U

typedef struct
{
    uint32_t magic;
    uint16_t version;
    uint16_t size;
    float zero_voltage[3];
    float gain_correction[3];
    uint16_t wavelength_nm;
    uint8_t display_mode;
    uint8_t mode;
    uint8_t wavelength_cal_count;
    uint8_t reserved[3];
    pm_wavelength_cal_point_t wavelength_cal[PM_WAVELENGTH_CAL_MAX_POINTS];
    uint32_t crc32;
} pm_storage_record_t;

_Static_assert(sizeof(pm_storage_record_t) == PM_STORAGE_SIZE, "AT24C02 record layout changed");
_Static_assert(PM_STORAGE_ADDR + PM_STORAGE_SIZE <= 255U, "AT24C02 allocation exceeds user area");

static uint32_t pm_crc32(const uint8_t *data, size_t length)
{
    uint32_t crc = 0xFFFFFFFFUL;
    for (size_t i = 0U; i < length; i++)
    {
        crc ^= data[i];
        for (uint8_t bit = 0U; bit < 8U; bit++)
        {
            crc = (crc & 1U) ? (crc >> 1U) ^ 0xEDB88320UL : (crc >> 1U);
        }
    }
    return ~crc;
}

static bool pm_record_valid(const pm_storage_record_t *record)
{
    if (record->magic != PM_STORAGE_MAGIC || record->version != PM_STORAGE_VERSION ||
        record->size != sizeof(*record) || record->wavelength_nm < PM_WAVELENGTH_MIN_NM ||
        record->wavelength_nm > PM_WAVELENGTH_MAX_NM || record->display_mode > PM_DISPLAY_MODE_DBM ||
        (record->mode != PM_MODE_AUTO && record->mode != PM_MODE_MANUAL) ||
        record->crc32 != pm_crc32((const uint8_t *)record, offsetof(pm_storage_record_t, crc32)))
    {
        return false;
    }
    for (uint32_t i = 0U; i < 3U; i++)
    {
        if (!isfinite(record->zero_voltage[i]) || !isfinite(record->gain_correction[i]) ||
            record->gain_correction[i] <= 0.0f)
        {
            return false;
        }
    }
    if(record->wavelength_cal_count>PM_WAVELENGTH_CAL_MAX_POINTS)return false;
    for(uint8_t i=0U;i<record->wavelength_cal_count;i++) if(record->wavelength_cal[i].wavelength_nm<PM_WAVELENGTH_MIN_NM||record->wavelength_cal[i].wavelength_nm>PM_WAVELENGTH_MAX_NM||!isfinite(record->wavelength_cal[i].correction_factor)||record->wavelength_cal[i].correction_factor<=0.0f)return false;
    return true;
}

bool PM_Storage_Load(pm_display_mode_t *display_mode)
{
    pm_storage_record_t record;
    at24cxx_read(PM_STORAGE_ADDR, (uint8_t *)&record, sizeof(record));
    if (!pm_record_valid(&record)) { return false; }

    pm_power_parameters_t parameters = {0};
    memcpy(parameters.zero_voltage, record.zero_voltage, sizeof(parameters.zero_voltage));
    memcpy(parameters.gain_correction, record.gain_correction, sizeof(parameters.gain_correction));
    if (!PM_SetPowerParameters(&parameters) || PM_SetWavelength(record.wavelength_nm) != record.wavelength_nm ||
        !PM_SetMode((pm_mode_t)record.mode))
    {
        return false;
    }
    PM_ResetWavelengthCalibration();
    for(uint8_t i=0U;i<record.wavelength_cal_count;i++) if(!PM_SetWavelengthCalibrationPoint(record.wavelength_cal[i].wavelength_nm,record.wavelength_cal[i].correction_factor)) return false;
    if (display_mode != NULL) { *display_mode = (pm_display_mode_t)record.display_mode; }
    return true;
}

bool PM_Storage_Save(pm_display_mode_t display_mode)
{
    if (display_mode > PM_DISPLAY_MODE_DBM) { return false; }
    const pm_measurement_t *measurement = PM_GetMeasurement();
    const pm_power_parameters_t *parameters = PM_GetPowerParameters();
    pm_storage_record_t record = {
        .magic = PM_STORAGE_MAGIC,
        .version = PM_STORAGE_VERSION,
        .size = sizeof(pm_storage_record_t),
        .wavelength_nm = measurement->wavelength_nm,
        .display_mode = (uint8_t)display_mode,
        .mode = (uint8_t)measurement->mode
    };
    memcpy(record.zero_voltage, parameters->zero_voltage, sizeof(record.zero_voltage));
    memcpy(record.gain_correction, parameters->gain_correction, sizeof(record.gain_correction));
    record.wavelength_cal_count=PM_GetWavelengthCalibrationPoints(record.wavelength_cal,PM_WAVELENGTH_CAL_MAX_POINTS);
    record.crc32 = pm_crc32((const uint8_t *)&record, offsetof(pm_storage_record_t, crc32));

    at24cxx_write(PM_STORAGE_ADDR, (uint8_t *)&record, sizeof(record));
    pm_storage_record_t verify;
    at24cxx_read(PM_STORAGE_ADDR, (uint8_t *)&verify, sizeof(verify));
    return memcmp(&record, &verify, sizeof(record)) == 0;
}
