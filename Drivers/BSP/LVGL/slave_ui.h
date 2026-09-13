#ifndef __SLAVE_UI_H
#define __SLAVE_UI_H

#include <stdbool.h>
#include <stdint.h>
#include "lvgl.h"

void slave_ui_create(lv_obj_t *parent);

void slave_ui_set_command_callback(void (*callback)(const char *command));
void slave_ui_set_pynq_status(const char *status);
void slave_ui_set_test_result(const char *result);
void slave_ui_set_capture_state(const char *state);
void slave_ui_append_log(const char *line);

void slave_ui_set_rx_state(bool carrier_detected,
                           bool selected_call,
                           bool group_call,
                           int16_t rssi_dbm,
                           uint8_t af_level);
void slave_ui_set_sms(const char *text, uint8_t sender_id, bool group_call);
void slave_ui_set_battery(uint16_t millivolts, uint8_t percent);
void slave_ui_set_waveform(const uint16_t *samples, uint16_t count);
void slave_ui_reset_waveform(void);

/* Set before slave_ui_create() to restore the persisted display selection. */
void slave_ui_set_power_meter_display_mode(uint8_t display_mode);

uint8_t slave_ui_get_station_id(void);
bool slave_ui_group_enabled(void);

#endif
