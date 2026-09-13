#include "./BSP/LVGL/slave_ui.h"
#include "./BSP/ADC/adc.h"
#include "power_meter.h"
#include "power_meter_storage.h"

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "stm32h7xx_hal.h"

extern const lv_font_t lv_font_chinese_14;

#define SLAVE_STATION_COUNT       8U
#define SMS_TEXT_MAX              96U
#define PYNQ_TEXT_MAX             128U
#define PYNQ_LOG_LINES            20U
#define PYNQ_LOG_LINE_MAX         160U
#define WAVE_POINT_COUNT          160U
#define ADC_REF_MV                3300U
#define ADC_FULL_SCALE            65535U
#define WAVE_INPUT_CAL_NUM        5U
#define WAVE_INPUT_CAL_DEN        6U
#define WAVE_SAMPLE_RATE_HZ       (adc_dma_get_sample_rate_hz())
#define WAVE_FREQ_MAX_HZ          10000U
#define WAVE_LOW_FREQ_LIMIT_HZ    2200U
#define WAVE_LOW_FREQ_AVG_SHIFT   5U
#define WAVE_FREQ_DISPLAY_MS      900U
#define WAVE_LOW_FREQ_BUF_SIZE    4096U
#define WAVE_LOW_FREQ_ESTIMATE_MS 80U
#define WAVE_LOW_FREQ_MIN_HZ      100U
#define WAVE_UI_UPDATE_MS         200U
#define WAVE_DRAW_ENABLE          0
#define WAVE_MIN_SPAN_RAW         256U

typedef enum {
    PAGE_POWER = 0,
    /* Legacy page renderers remain compiled below but have no visible entry. */
    PAGE_HOME,
    PAGE_RX,
    PAGE_SMS,
    PAGE_SETUP,
    PAGE_SCOPE,
    PAGE_COUNT
} slave_page_t;

typedef struct {
    uint8_t station_id;
    bool group_enabled;
    bool carrier_detected;
    bool selected_call;
    bool group_call;
    bool muted;
    uint8_t volume;
    uint8_t squelch;
    uint8_t af_level;
    int16_t rssi_dbm;
    uint16_t battery_mv;
    uint8_t battery_percent;
    uint32_t packet_count;
    uint8_t last_sender;
    char sms_text[SMS_TEXT_MAX];
    char pynq_status[PYNQ_TEXT_MAX];
    char test_result[PYNQ_TEXT_MAX];
    char capture_state[PYNQ_TEXT_MAX];
    char log_lines[PYNQ_LOG_LINES][PYNQ_LOG_LINE_MAX];
    uint8_t log_count;
    uint16_t wave_min;
    uint16_t wave_max;
    uint16_t wave_avg;
    uint16_t wave_pp_mv;
    uint32_t wave_freq_x10;
    uint32_t wave_freq_calc_x10;
    uint64_t wave_freq_sum_x10;
    uint32_t wave_freq_last_ms;
    uint16_t wave_freq_sample_count;
    uint32_t wave_low_freq_last_ms;
    uint32_t wave_frame_count;
    uint64_t wave_abs_sample;
    uint64_t wave_last_cross_x100;
    uint64_t wave_period_avg_x100;
    bool wave_freq_valid;
    bool wave_freq_calc_valid;
    bool wave_above_high;
} slave_ui_state_t;

typedef struct {
    lv_obj_t *root;
    lv_obj_t *top;
    lv_obj_t *status;
    lv_obj_t *content;
    lv_obj_t *nav_btn[PAGE_COUNT];
    lv_obj_t *label_rx;
    lv_obj_t *label_id;
    lv_obj_t *label_batt;
    lv_obj_t *label_rssi;
    lv_obj_t *label_uptime;
    lv_obj_t *label_status;
    lv_obj_t *label_station_value;
    lv_obj_t *label_group_value;
    lv_obj_t *label_wave_info;
    lv_obj_t *label_pm_debug;
    lv_obj_t *label_pm_power;
    lv_obj_t *label_pm_wavelength;
    lv_obj_t *label_pm_main;
    lv_obj_t *label_pm_range;
    lv_obj_t *label_pm_afe;
    lv_obj_t *label_pm_status;
    lv_obj_t *label_pm_hint;
    lv_obj_t *btn_pm_linear;
    lv_obj_t *btn_pm_unit;
    lv_obj_t *btn_pm_auto;
    lv_obj_t *btn_pm_range[3];
    lv_obj_t *zero_confirm_overlay;
    lv_obj_t *cal_overlay;
    lv_obj_t *label_cal_input;
    lv_obj_t *label_cal_wave;
    lv_obj_t *label_cal_status;
    lv_obj_t *label_cal_afe;
    lv_obj_t *label_cal_raw_power;
    lv_obj_t *label_cal_current_range;
    lv_obj_t *label_cal_gain;
    lv_obj_t *label_cal_points;
    lv_obj_t *btn_cal_range[3];
    lv_obj_t *label_pynq_status;
    lv_obj_t *label_test_result;
    lv_obj_t *label_capture_state;
    lv_obj_t *label_sms_body;
    lv_obj_t *label_log;
    lv_obj_t *wave_line;
    lv_obj_t *bar_af;
    lv_obj_t *bar_rssi;
    lv_point_precise_t wave_points[WAVE_POINT_COUNT];
    slave_page_t page;
    bool pm_show_dbm;
    bool pm_show_uW;
    bool zero_save_handled;
    bool zero_save_failed;
    pm_range_t cal_range;
    char cal_input[16];
    pm_power_parameters_t zero_backup;
    pm_power_parameters_t cal_backup;
    slave_ui_state_t state;
} slave_ui_t;

static slave_ui_t g_ui;
static pm_display_mode_t g_pm_start_display_mode = PM_DISPLAY_MODE_MW;
static uint16_t g_wave_low_freq_buf[WAVE_LOW_FREQ_BUF_SIZE];
static uint16_t g_wave_low_freq_len = 0U;
static void (*g_command_callback)(const char *command) = NULL;

#define C_BG        lv_color_hex(0x0F1419)
#define C_PANEL     lv_color_hex(0x172028)
#define C_PANEL_2   lv_color_hex(0x1F2A33)
#define C_LINE      lv_color_hex(0x344450)
#define C_TEXT      lv_color_hex(0xF4F7FA)
#define C_MUTED     lv_color_hex(0x9DAAB5)
#define C_ACCENT    lv_color_hex(0x42C6E8)
#define C_WARN      lv_color_hex(0xF2C24B)
#define C_OK        lv_color_hex(0x64D48E)
#define C_BAD       lv_color_hex(0xE96666)

static void render_page(slave_page_t page);
static void command_button_event(lv_event_t *event);
static void update_pynq_labels(void);
static void push_frequency_sample(uint32_t freq_x10);
static void append_low_frequency_samples(const uint16_t *samples, uint16_t count);
static uint32_t estimate_low_frequency_x10(uint32_t hint_freq_x10);
static void reset_frequency_display(void);
static void update_power_meter_ui(lv_timer_t *timer);
static void pm_manual_event(lv_event_t *event);
static void pm_zero_ui_service(void);
static void pm_cal_update_dialog(void);

static lv_obj_t *make_obj(lv_obj_t *parent, int32_t x, int32_t y, int32_t w, int32_t h, lv_color_t bg)
{
    lv_obj_t *obj = lv_obj_create(parent);
    lv_obj_set_pos(obj, x, y);
    lv_obj_set_size(obj, w, h);
    lv_obj_set_style_bg_color(obj, bg, 0);
    lv_obj_set_style_bg_opa(obj, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(obj, 0, 0);
    lv_obj_set_style_pad_all(obj, 0, 0);
    lv_obj_clear_flag(obj, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scrollbar_mode(obj, LV_SCROLLBAR_MODE_OFF);
    return obj;
}

static lv_obj_t *make_label(lv_obj_t *parent,
                            const char *text,
                            int32_t x,
                            int32_t y,
                            const lv_font_t *font,
                            lv_color_t color)
{
    lv_obj_t *label = lv_label_create(parent);
    lv_label_set_text(label, text);
    lv_obj_set_pos(label, x, y);
    lv_obj_set_style_text_color(label, color, 0);
    if (font != NULL) {
        lv_obj_set_style_text_font(label, font, 0);
    }
    return label;
}

static lv_obj_t *make_panel(lv_obj_t *parent, int32_t x, int32_t y, int32_t w, int32_t h, const char *title)
{
    lv_obj_t *panel = make_obj(parent, x, y, w, h, C_PANEL);
    lv_obj_set_style_radius(panel, 6, 0);
    lv_obj_set_style_border_width(panel, 1, 0);
    lv_obj_set_style_border_color(panel, C_LINE, 0);
    if (title != NULL) {
        make_label(panel, title, 10, 8, &lv_font_chinese_14, C_MUTED);
        make_obj(panel, 10, 31, w - 20, 1, C_LINE);
    }
    return panel;
}

static lv_obj_t *make_button(lv_obj_t *parent,
                             const char *text,
                             int32_t x,
                             int32_t y,
                             int32_t w,
                             int32_t h,
                             lv_color_t bg,
                             lv_event_cb_t cb)
{
    lv_obj_t *btn = lv_button_create(parent);
    lv_obj_set_pos(btn, x, y);
    lv_obj_set_size(btn, w, h);
    lv_obj_set_style_bg_color(btn, bg, 0);
    lv_obj_set_style_radius(btn, 5, 0);
    lv_obj_set_style_shadow_width(btn, 0, 0);
    lv_obj_set_style_border_width(btn, 1, 0);
    lv_obj_set_style_border_color(btn, C_LINE, 0);
    lv_obj_clear_flag(btn, LV_OBJ_FLAG_SCROLLABLE);
    if (cb != NULL) {
        lv_obj_add_event_cb(btn, cb, LV_EVENT_CLICKED, NULL);
    }

    lv_obj_t *label = lv_label_create(btn);
    lv_label_set_text(label, text);
    lv_obj_set_style_text_color(label, C_TEXT, 0);
    lv_obj_set_style_text_font(label, &lv_font_chinese_14, 0);
    lv_obj_center(label);
    return btn;
}

static void set_button_selected(lv_obj_t *button, bool selected)
{
    if (button == NULL) { return; }
    lv_obj_set_style_bg_color(button, selected ? C_ACCENT : C_PANEL_2, 0);
    lv_obj_t *label = lv_obj_get_child(button, 0);
    if (label != NULL) {
        lv_obj_set_style_text_color(label, selected ? C_BG : C_TEXT, 0);
    }
}

static lv_obj_t *make_value_card(lv_obj_t *parent,
                                 int32_t x,
                                 int32_t y,
                                 int32_t w,
                                 int32_t h,
                                 const char *name,
                                 const char *value,
                                 lv_color_t color)
{
    lv_obj_t *panel = make_panel(parent, x, y, w, h, NULL);
    make_label(panel, name, 10, 8, &lv_font_montserrat_14, C_MUTED);
    lv_obj_t *label = make_label(panel, value, 10, 32, &lv_font_montserrat_18, color);
    lv_obj_set_width(label, w - 20);
    lv_label_set_long_mode(label, LV_LABEL_LONG_CLIP);
    return panel;
}

static lv_obj_t *make_row(lv_obj_t *parent, int32_t y, const char *name, const char *value, lv_color_t color)
{
    make_label(parent, name, 12, y, &lv_font_montserrat_14, C_MUTED);
    lv_obj_t *label = make_label(parent, value, 118, y, &lv_font_montserrat_14, color);
    lv_obj_set_width(label, 170);
    lv_label_set_long_mode(label, LV_LABEL_LONG_CLIP);
    return label;
}

static uint16_t raw_to_mv(uint16_t raw)
{
    uint32_t adc_mv = (uint32_t)(((uint64_t)raw * ADC_REF_MV + (ADC_FULL_SCALE / 2U)) / ADC_FULL_SCALE);

    return (uint16_t)((adc_mv * WAVE_INPUT_CAL_NUM + (WAVE_INPUT_CAL_DEN / 2U)) / WAVE_INPUT_CAL_DEN);
}

static int32_t rssi_to_percent(int16_t dbm)
{
    if (dbm <= -120) {
        return 0;
    }
    if (dbm >= -40) {
        return 100;
    }
    return ((int32_t)dbm + 120) * 100 / 80;
}

static const char *rx_status_text(void)
{
    if (!g_ui.state.carrier_detected) {
        return "IDLE";
    }
    if (g_ui.state.selected_call) {
        return g_ui.state.group_call ? "GROUP" : "CALL";
    }
    return "CARRIER";
}

static void update_top(void)
{
    char buf[40];
    lv_color_t rx_color = g_ui.state.selected_call ? C_OK :
                          (g_ui.state.carrier_detected ? C_WARN : C_BAD);

    if (g_ui.label_rx != NULL) {
        lv_label_set_text(g_ui.label_rx, rx_status_text());
        lv_obj_set_style_text_color(g_ui.label_rx, rx_color, 0);
    }

    if (g_ui.label_id != NULL) {
        snprintf(buf, sizeof(buf), "ID:%u", g_ui.state.station_id);
        lv_label_set_text(g_ui.label_id, buf);
    }

    if (g_ui.label_batt != NULL) {
        snprintf(buf, sizeof(buf), "BAT:%u%%", g_ui.state.battery_percent);
        lv_label_set_text(g_ui.label_batt, buf);
        lv_obj_set_style_text_color(g_ui.label_batt,
                                    g_ui.state.battery_percent < 20U ? C_BAD : C_OK,
                                    0);
    }

    if (g_ui.label_rssi != NULL) {
        snprintf(buf, sizeof(buf), "%ddBm", (int)g_ui.state.rssi_dbm);
        lv_label_set_text(g_ui.label_rssi, buf);
    }

    if (g_ui.label_station_value != NULL) {
        snprintf(buf, sizeof(buf), "%u", g_ui.state.station_id);
        lv_label_set_text(g_ui.label_station_value, buf);
    }

    if (g_ui.label_group_value != NULL) {
        lv_label_set_text(g_ui.label_group_value, g_ui.state.group_enabled ? "ON" : "OFF");
        lv_obj_set_style_text_color(g_ui.label_group_value,
                                    g_ui.state.group_enabled ? C_OK : C_WARN,
                                    0);
    }

    if (g_ui.bar_af != NULL) {
        lv_bar_set_value(g_ui.bar_af, g_ui.state.af_level, LV_ANIM_OFF);
    }

    if (g_ui.bar_rssi != NULL) {
        lv_bar_set_value(g_ui.bar_rssi, rssi_to_percent(g_ui.state.rssi_dbm), LV_ANIM_OFF);
    }
}

static void station_minus_event(lv_event_t *event)
{
    (void)event;
    if (g_ui.state.station_id == 0U) {
        g_ui.state.station_id = SLAVE_STATION_COUNT - 1U;
    } else {
        g_ui.state.station_id--;
    }
    update_top();
}

static void station_plus_event(lv_event_t *event)
{
    (void)event;
    g_ui.state.station_id = (uint8_t)((g_ui.state.station_id + 1U) % SLAVE_STATION_COUNT);
    update_top();
}

static void group_toggle_event(lv_event_t *event)
{
    (void)event;
    g_ui.state.group_enabled = !g_ui.state.group_enabled;
    update_top();
    render_page(PAGE_SETUP);
}

static void mute_toggle_event(lv_event_t *event)
{
    (void)event;
    g_ui.state.muted = !g_ui.state.muted;
    render_page(g_ui.page);
}

static void volume_minus_event(lv_event_t *event)
{
    (void)event;
    if (g_ui.state.volume >= 5U) {
        g_ui.state.volume -= 5U;
    }
    render_page(g_ui.page);
}

static void volume_plus_event(lv_event_t *event)
{
    (void)event;
    if (g_ui.state.volume <= 95U) {
        g_ui.state.volume += 5U;
    }
    render_page(g_ui.page);
}

static void sms_clear_event(lv_event_t *event)
{
    (void)event;
    g_ui.state.sms_text[0] = '\0';
    render_page(PAGE_SMS);
}

static void simulate_call_event(lv_event_t *event)
{
    (void)event;
    g_ui.state.carrier_detected = true;
    g_ui.state.selected_call = true;
    g_ui.state.group_call = false;
    g_ui.state.rssi_dbm = -62;
    g_ui.state.af_level = 58;
    update_top();
    render_page(PAGE_HOME);
}

static void command_button_event(lv_event_t *event)
{
    const char *command = (const char *)lv_event_get_user_data(event);

    if (command == NULL) {
        return;
    }

    snprintf(g_ui.state.capture_state, sizeof(g_ui.state.capture_state), "%s sent", command);
    slave_ui_append_log(command);
    update_pynq_labels();

    if (g_command_callback != NULL) {
        g_command_callback(command);
    }
}

static void update_nav(void)
{
    for (uint8_t i = 0; i < PAGE_COUNT; i++) {
        if (g_ui.nav_btn[i] == NULL) { continue; }
        lv_color_t bg = (i == (uint8_t)g_ui.page) ? C_ACCENT : C_PANEL_2;
        lv_color_t fg = (i == (uint8_t)g_ui.page) ? C_BG : C_TEXT;
        lv_obj_set_style_bg_color(g_ui.nav_btn[i], bg, 0);
        lv_obj_set_style_text_color(lv_obj_get_child(g_ui.nav_btn[i], 0), fg, 0);
    }
}

static void update_pynq_labels(void)
{
    if (g_ui.label_pynq_status != NULL) {
        lv_label_set_text(g_ui.label_pynq_status,
                          g_ui.state.pynq_status[0] ? g_ui.state.pynq_status : "BOOT WAIT");
    }

    if (g_ui.label_test_result != NULL) {
        lv_label_set_text(g_ui.label_test_result,
                          g_ui.state.test_result[0] ? g_ui.state.test_result : "--");
    }

    if (g_ui.label_capture_state != NULL) {
        lv_label_set_text(g_ui.label_capture_state,
                          g_ui.state.capture_state[0] ? g_ui.state.capture_state : "IDLE");
    }

    if (g_ui.label_sms_body != NULL) {
        lv_label_set_text(g_ui.label_sms_body,
                          g_ui.state.sms_text[0] ? g_ui.state.sms_text : "(waiting for PYNQ SMS)");
        lv_obj_set_style_text_color(g_ui.label_sms_body,
                                    g_ui.state.sms_text[0] ? C_TEXT : C_MUTED,
                                    0);
    }

    if (g_ui.label_log != NULL) {
        char joined[(PYNQ_LOG_LINES * (PYNQ_LOG_LINE_MAX + 1U))];
        size_t pos = 0U;

        joined[0] = '\0';
        for (uint8_t i = 0; i < g_ui.state.log_count; i++) {
            int written = snprintf(&joined[pos], sizeof(joined) - pos,
                                   "%s%s",
                                   g_ui.state.log_lines[i],
                                   (i + 1U < g_ui.state.log_count) ? "\n" : "");
            if (written <= 0) {
                break;
            }
            if ((size_t)written >= sizeof(joined) - pos) {
                pos = sizeof(joined) - 1U;
                break;
            }
            pos += (size_t)written;
        }

        if (joined[0] == '\0') {
            snprintf(joined, sizeof(joined), "waiting for PYNQ...");
        }
        lv_label_set_text(g_ui.label_log, joined);
    }
}

static void render_home(void)
{
    lv_obj_t *c = g_ui.content;
    char value[40];

    make_value_card(c, 8, 8, 186, 76, "Frequency", "35.000 MHz", C_ACCENT);
    make_value_card(c, 202, 8, 186, 76, "RX State", rx_status_text(),
                    g_ui.state.selected_call ? C_OK : C_WARN);

    snprintf(value, sizeof(value), "S%u%s", g_ui.state.station_id,
             g_ui.state.group_enabled ? " + GRP" : "");
    make_value_card(c, 396, 8, 186, 76, "Address", value, C_TEXT);

    snprintf(value, sizeof(value), "%u%%", g_ui.state.battery_percent);
    make_value_card(c, 590, 8, 202, 76, "Battery", value,
                    g_ui.state.battery_percent < 20U ? C_BAD : C_OK);

    lv_obj_t *rx = make_panel(c, 8, 94, 380, 250, "Receive Monitor");
    g_ui.bar_rssi = lv_bar_create(rx);
    lv_obj_set_pos(g_ui.bar_rssi, 18, 58);
    lv_obj_set_size(g_ui.bar_rssi, 248, 22);
    lv_bar_set_range(g_ui.bar_rssi, 0, 100);
    lv_obj_set_style_bg_color(g_ui.bar_rssi, C_PANEL_2, 0);
    lv_obj_set_style_bg_color(g_ui.bar_rssi, C_ACCENT, LV_PART_INDICATOR);
    make_label(rx, "RSSI", 286, 59, &lv_font_montserrat_14, C_MUTED);

    g_ui.bar_af = lv_bar_create(rx);
    lv_obj_set_pos(g_ui.bar_af, 18, 112);
    lv_obj_set_size(g_ui.bar_af, 248, 22);
    lv_bar_set_range(g_ui.bar_af, 0, 100);
    lv_obj_set_style_bg_color(g_ui.bar_af, C_PANEL_2, 0);
    lv_obj_set_style_bg_color(g_ui.bar_af, C_OK, LV_PART_INDICATOR);
    make_label(rx, "AF", 286, 113, &lv_font_montserrat_14, C_MUTED);

    make_row(rx, 156, "Demod", "AM audio", C_TEXT);
    make_row(rx, 184, "Audio out", g_ui.state.muted ? "Muted" : "Headphone", g_ui.state.muted ? C_WARN : C_OK);
    make_row(rx, 212, "Data", "AFSK 100bps", C_TEXT);

    lv_obj_t *msg = make_panel(c, 396, 94, 396, 250, "Last Message");
    if (g_ui.state.group_call) {
        snprintf(value, sizeof(value), "From: ALL (group)");
    } else {
        snprintf(value, sizeof(value), "From: S%u", g_ui.state.last_sender);
    }
    make_label(msg, value, 14, 48, &lv_font_montserrat_14, C_MUTED);

    lv_obj_t *sms = make_label(msg,
                               g_ui.state.sms_text[0] ? g_ui.state.sms_text : "(no received SMS)",
                               14, 82,
                               &lv_font_montserrat_18,
                               g_ui.state.sms_text[0] ? C_TEXT : C_MUTED);
    lv_obj_set_width(sms, 360);
    lv_label_set_long_mode(sms, LV_LABEL_LONG_WRAP);
    snprintf(value, sizeof(value), "%lu", (unsigned long)g_ui.state.packet_count);
    make_row(msg, 190, "Packets", value, C_TEXT);
    make_button(msg, g_ui.state.muted ? "UNMUTE" : "MUTE", 14, 202, 110, 34,
                g_ui.state.muted ? C_WARN : C_PANEL_2, mute_toggle_event);

    update_top();
}

static void render_rx(void)
{
    lv_obj_t *c = g_ui.content;
    char value[40];

    lv_obj_t *left = make_panel(c, 8, 8, 384, 336, "Voice Receive");
    make_row(left, 48, "Carrier", g_ui.state.carrier_detected ? "Detected" : "Idle",
             g_ui.state.carrier_detected ? C_OK : C_MUTED);
    make_row(left, 78, "Selected", g_ui.state.selected_call ? "Yes" : "No",
             g_ui.state.selected_call ? C_OK : C_WARN);
    make_row(left, 108, "Call type", g_ui.state.group_call ? "Group" : "Station", C_TEXT);
    make_row(left, 138, "Headphone", g_ui.state.muted ? "Muted" : "Enabled",
             g_ui.state.muted ? C_WARN : C_OK);

    snprintf(value, sizeof(value), "%u%%", g_ui.state.volume);
    make_row(left, 168, "Volume", value, C_TEXT);
    make_button(left, "-", 18, 204, 58, 42, C_PANEL_2, volume_minus_event);
    make_button(left, "+", 90, 204, 58, 42, C_PANEL_2, volume_plus_event);
    make_button(left, g_ui.state.muted ? "UNMUTE" : "MUTE", 166, 204, 120, 42,
                g_ui.state.muted ? C_WARN : C_PANEL_2, mute_toggle_event);

    lv_obj_t *bar = lv_bar_create(left);
    lv_obj_set_pos(bar, 18, 272);
    lv_obj_set_size(bar, 320, 24);
    lv_bar_set_range(bar, 0, 100);
    lv_bar_set_value(bar, g_ui.state.af_level, LV_ANIM_OFF);
    lv_obj_set_style_bg_color(bar, C_PANEL_2, 0);
    lv_obj_set_style_bg_color(bar, C_OK, LV_PART_INDICATOR);
    make_label(left, "AF level", 18, 250, &lv_font_montserrat_14, C_MUTED);

    lv_obj_t *right = make_panel(c, 400, 8, 392, 336, "Receiver Chain");
    make_row(right, 48, "RF band", "30-40 MHz", C_OK);
    make_row(right, 78, "RX freq", "35.000 MHz", C_ACCENT);
    make_row(right, 108, "Antenna", "<= 1m", C_OK);
    make_row(right, 138, "Demod", "AM envelope", C_TEXT);
    make_row(right, 168, "SMS modem", "AFSK 1200/2200", C_TEXT);
    make_row(right, 198, "Squelch", "Manual", C_WARN);
    make_row(right, 228, "Distance", ">= 5m target", C_WARN);
    make_button(right, "SIM CALL", 18, 276, 130, 42, C_ACCENT, simulate_call_event);
}

static void render_sms(void)
{
    lv_obj_t *c = g_ui.content;
    char value[48];
    int32_t content_w = lv_obj_get_width(c);
    int32_t content_h = lv_obj_get_height(c);
    if (content_w <= 0) { content_w = 800; }
    if (content_h <= 0) { content_h = 340; }

    /* Three-column layout: Control | SMS Inbox | Log */
    int32_t pad = 6;
    int32_t ctrl_w = 180;
    int32_t inbox_w = 260;
    int32_t log_w = content_w - ctrl_w - inbox_w - pad * 4;
    if (log_w < 200) { log_w = 200; }

    int32_t ctrl_x = pad;
    int32_t inbox_x = ctrl_x + ctrl_w + pad;
    int32_t log_x = inbox_x + inbox_w + pad;
    int32_t panel_h = content_h - pad * 2;
    int32_t log_inner_w = log_w - 20;

    /* ---- PYNQ Control (left) ---- */
    lv_obj_t *control = make_panel(c, ctrl_x, pad, ctrl_w, panel_h, "PYNQ Control");
    lv_obj_t *btn;
    make_label(control, "UART1 115200", 10, 38, &lv_font_montserrat_14, C_MUTED);
    btn = make_button(control, "STATUS", 10, 68, 72, 36, C_PANEL_2, NULL);
    lv_obj_add_event_cb(btn, command_button_event, LV_EVENT_CLICKED, "STATUS");
    btn = make_button(control, "TEST", 90, 68, 72, 36, C_PANEL_2, NULL);
    lv_obj_add_event_cb(btn, command_button_event, LV_EVENT_CLICKED, "TEST");
    btn = make_button(control, "CAPTURE", 10, 116, 152, 42, C_ACCENT, NULL);
    lv_obj_add_event_cb(btn, command_button_event, LV_EVENT_CLICKED, "CAPTURE");
    make_button(control, "CLEAR", 10, panel_h - 58, 80, 36, C_WARN, sms_clear_event);

    lv_obj_t *state = make_panel(control, 10, 172, ctrl_w - 28, 72, NULL);
    g_ui.label_capture_state = make_label(state, "IDLE", 8, 8, &lv_font_montserrat_14, C_TEXT);
    lv_obj_set_width(g_ui.label_capture_state, ctrl_w - 44);
    lv_label_set_long_mode(g_ui.label_capture_state, LV_LABEL_LONG_CLIP);
    g_ui.label_test_result = make_label(state, "--", 8, 34, &lv_font_montserrat_14, C_ACCENT);
    lv_obj_set_width(g_ui.label_test_result, ctrl_w - 44);
    lv_label_set_long_mode(g_ui.label_test_result, LV_LABEL_LONG_CLIP);

    /* ---- SMS Inbox (middle) ---- */
    lv_obj_t *inbox = make_panel(c, inbox_x, pad, inbox_w, panel_h, "SMS Characters");
    if (g_ui.state.group_call) {
        snprintf(value, sizeof(value), "Addr: ALL (group)");
    } else {
        snprintf(value, sizeof(value), "Addr: S%u", g_ui.state.last_sender);
    }
    make_label(inbox, value, 12, 40, &lv_font_montserrat_14, C_MUTED);

    lv_obj_t *body = make_panel(inbox, 12, 70, inbox_w - 32, panel_h - 88, NULL);
    g_ui.label_sms_body = make_label(body,
                                     g_ui.state.sms_text[0] ? g_ui.state.sms_text : "(waiting for PYNQ SMS)",
                                     10, 10,
                                     &lv_font_montserrat_18,
                                     g_ui.state.sms_text[0] ? C_TEXT : C_MUTED);
    lv_obj_set_width(g_ui.label_sms_body, inbox_w - 52);
    lv_label_set_long_mode(g_ui.label_sms_body, LV_LABEL_LONG_WRAP);

    /* ---- PYNQ Log (right, scrollable, wide) ---- */
    lv_obj_t *log = make_panel(c, log_x, pad, log_w, panel_h, "PYNQ Log");
    g_ui.label_pynq_status = make_label(log, "BOOT WAIT", 10, 38, &lv_font_montserrat_14, C_ACCENT);
    lv_obj_set_width(g_ui.label_pynq_status, log_inner_w);
    lv_label_set_long_mode(g_ui.label_pynq_status, LV_LABEL_LONG_CLIP);

    /* Scrollable log body — label auto-expands, parent scrolls */
    lv_obj_t *log_body = lv_obj_create(log);
    lv_obj_set_pos(log_body, 8, 68);
    lv_obj_set_size(log_body, log_inner_w + 8, panel_h - 84);
    lv_obj_set_style_bg_color(log_body, C_PANEL, 0);
    lv_obj_set_style_bg_opa(log_body, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(log_body, 0, 0);
    lv_obj_set_style_pad_all(log_body, 4, 0);
    lv_obj_set_scrollbar_mode(log_body, LV_SCROLLBAR_MODE_AUTO);
    lv_obj_add_flag(log_body, LV_OBJ_FLAG_SCROLLABLE);

    g_ui.label_log = make_label(log_body, "waiting for PYNQ...", 4, 4, &lv_font_montserrat_14, C_TEXT);
    lv_obj_set_width(g_ui.label_log, log_inner_w);
    lv_label_set_long_mode(g_ui.label_log, LV_LABEL_LONG_WRAP);

    update_pynq_labels();
}

static void render_setup(void)
{
    lv_obj_t *c = g_ui.content;
    char value[40];

    lv_obj_t *addr = make_panel(c, 8, 8, 384, 336, "Station Address");
    make_label(addr, "Station ID", 20, 54, &lv_font_montserrat_14, C_MUTED);
    g_ui.label_station_value = make_label(addr, "0", 154, 48, &lv_font_montserrat_18, C_ACCENT);
    make_button(addr, "-", 20, 92, 72, 48, C_PANEL_2, station_minus_event);
    make_button(addr, "+", 110, 92, 72, 48, C_PANEL_2, station_plus_event);
    make_label(addr, "Range: S0-S7", 204, 108, &lv_font_montserrat_14, C_MUTED);

    make_label(addr, "Accept group call", 20, 174, &lv_font_montserrat_14, C_MUTED);
    g_ui.label_group_value = make_label(addr, "ON", 184, 174, &lv_font_montserrat_18, C_OK);
    make_button(addr, "TOGGLE", 20, 214, 140, 42, C_PANEL_2, group_toggle_event);
    lv_obj_t *note = make_label(addr, "Save ID in RF/config layer.", 20, 286, &lv_font_montserrat_14, C_WARN);
    lv_obj_set_width(note, 340);
    lv_label_set_long_mode(note, LV_LABEL_LONG_WRAP);

    lv_obj_t *sys = make_panel(c, 400, 8, 392, 336, "System Checklist");
    snprintf(value, sizeof(value), "%umV / %u%%", g_ui.state.battery_mv, g_ui.state.battery_percent);
    make_row(sys, 48, "Battery", value, g_ui.state.battery_percent < 20U ? C_BAD : C_OK);
    make_row(sys, 78, "Power", "Battery only", C_OK);
    make_row(sys, 108, "Audio out", "Headphone", C_OK);
    make_row(sys, 138, "RX freq", "Match master", C_ACCENT);
    make_row(sys, 168, "Direct test", "20dB attenuator", C_WARN);
    make_row(sys, 198, "Antenna", "<= 1m", C_OK);
    make_row(sys, 228, "RF decode", "TODO", C_WARN);
    make_row(sys, 258, "SMS parser", "TODO", C_WARN);
}

static void update_waveform_obj(void)
{
    static uint32_t last_update_ms = 0U;
    char info[112];
    char freq[24];
    uint32_t now = HAL_GetTick();

#if WAVE_DRAW_ENABLE
    if (g_ui.wave_line != NULL) {
        lv_line_set_points_mutable(g_ui.wave_line, g_ui.wave_points, WAVE_POINT_COUNT);
    }
#endif

    if (g_ui.label_wave_info != NULL) {
        if (last_update_ms != 0U && (now - last_update_ms) < WAVE_UI_UPDATE_MS) {
            return;
        }
        last_update_ms = now;

        if (g_ui.state.wave_freq_valid) {
            snprintf(freq, sizeof(freq), "%lu.%luHz",
                     (unsigned long)(g_ui.state.wave_freq_x10 / 10U),
                     (unsigned long)(g_ui.state.wave_freq_x10 % 10U));
        } else {
            snprintf(freq, sizeof(freq), "--.-Hz");
        }

        snprintf(info, sizeof(info), "Freq %s  Vpp %umV  Min %umV  Max %umV  Frames %lu",
                 freq,
                 g_ui.state.wave_pp_mv,
                 raw_to_mv(g_ui.state.wave_min),
                 raw_to_mv(g_ui.state.wave_max),
                 (unsigned long)g_ui.state.wave_frame_count);
        lv_label_set_text(g_ui.label_wave_info, info);
    }
}

static void push_frequency_sample(uint32_t freq_x10)
{
    uint32_t now = HAL_GetTick();

    g_ui.state.wave_freq_sum_x10 += freq_x10;
    if (g_ui.state.wave_freq_sample_count < 65535U) {
        g_ui.state.wave_freq_sample_count++;
    }

    if (g_ui.state.wave_freq_last_ms == 0U) {
        g_ui.state.wave_freq_last_ms = now;
    }

    if ((now - g_ui.state.wave_freq_last_ms) >= WAVE_FREQ_DISPLAY_MS &&
        g_ui.state.wave_freq_sample_count > 0U) {
        g_ui.state.wave_freq_x10 =
            (uint32_t)((g_ui.state.wave_freq_sum_x10 +
                        (g_ui.state.wave_freq_sample_count / 2U)) /
                       g_ui.state.wave_freq_sample_count);
        g_ui.state.wave_freq_valid = true;
        g_ui.state.wave_freq_sum_x10 = 0U;
        g_ui.state.wave_freq_sample_count = 0U;
        g_ui.state.wave_freq_last_ms = now;
    }
}

static void append_low_frequency_samples(const uint16_t *samples, uint16_t count)
{
    uint16_t copy_count = count;

    if (samples == NULL || count == 0U) {
        return;
    }

    if (copy_count >= WAVE_LOW_FREQ_BUF_SIZE) {
        memcpy(g_wave_low_freq_buf,
               &samples[copy_count - WAVE_LOW_FREQ_BUF_SIZE],
               WAVE_LOW_FREQ_BUF_SIZE * sizeof(uint16_t));
        g_wave_low_freq_len = WAVE_LOW_FREQ_BUF_SIZE;
        return;
    }

    if ((uint32_t)g_wave_low_freq_len + copy_count > WAVE_LOW_FREQ_BUF_SIZE) {
        uint16_t overflow = (uint16_t)((uint32_t)g_wave_low_freq_len + copy_count - WAVE_LOW_FREQ_BUF_SIZE);
        memmove(g_wave_low_freq_buf,
                &g_wave_low_freq_buf[overflow],
                (uint32_t)(g_wave_low_freq_len - overflow) * sizeof(uint16_t));
        g_wave_low_freq_len = (uint16_t)(g_wave_low_freq_len - overflow);
    }

    memcpy(&g_wave_low_freq_buf[g_wave_low_freq_len], samples, (uint32_t)copy_count * sizeof(uint16_t));
    g_wave_low_freq_len = (uint16_t)(g_wave_low_freq_len + copy_count);
}

static uint32_t estimate_low_frequency_x10(uint32_t hint_freq_x10)
{
    uint32_t now = HAL_GetTick();
    uint32_t hint_lag;
    uint32_t sample_rate_hz;
    uint16_t lag_min;
    uint16_t lag_max;
    uint16_t best_lag = 0U;
    int64_t best_score_avg = INT64_MIN;
    uint64_t sum = 0U;
    int32_t avg;

    if (hint_freq_x10 == 0U ||
        hint_freq_x10 > (WAVE_LOW_FREQ_LIMIT_HZ * 10U) ||
        hint_freq_x10 < (WAVE_LOW_FREQ_MIN_HZ * 10U) ||
        g_wave_low_freq_len < WAVE_LOW_FREQ_BUF_SIZE) {
        return 0U;
    }

    if (g_ui.state.wave_low_freq_last_ms != 0U &&
        (now - g_ui.state.wave_low_freq_last_ms) < WAVE_LOW_FREQ_ESTIMATE_MS) {
        return 0U;
    }

    g_ui.state.wave_low_freq_last_ms = now;

    sample_rate_hz = WAVE_SAMPLE_RATE_HZ;
    hint_lag = (sample_rate_hz * 10U + (hint_freq_x10 / 2U)) / hint_freq_x10;
    lag_min = (uint16_t)((hint_lag * 65U) / 100U);
    lag_max = (uint16_t)((hint_lag * 150U) / 100U);

    if (lag_min < (sample_rate_hz / WAVE_LOW_FREQ_LIMIT_HZ)) {
        lag_min = (uint16_t)(sample_rate_hz / WAVE_LOW_FREQ_LIMIT_HZ);
    }
    if (lag_max > (sample_rate_hz / WAVE_LOW_FREQ_MIN_HZ)) {
        lag_max = (uint16_t)(sample_rate_hz / WAVE_LOW_FREQ_MIN_HZ);
    }
    if (lag_max >= (WAVE_LOW_FREQ_BUF_SIZE - 8U)) {
        lag_max = WAVE_LOW_FREQ_BUF_SIZE - 8U;
    }
    if (lag_min >= lag_max) {
        return 0U;
    }

    for (uint16_t i = 0; i < WAVE_LOW_FREQ_BUF_SIZE; i++) {
        sum += g_wave_low_freq_buf[i];
    }
    avg = (int32_t)(sum / WAVE_LOW_FREQ_BUF_SIZE);

    for (uint16_t lag = lag_min; lag <= lag_max; lag++) {
        int64_t score = 0;
        int64_t score_avg;
        uint16_t pair_count = (uint16_t)(WAVE_LOW_FREQ_BUF_SIZE - lag);

        for (uint16_t i = 0; i < pair_count; i++) {
            int32_t a = (int32_t)g_wave_low_freq_buf[i] - avg;
            int32_t b = (int32_t)g_wave_low_freq_buf[i + lag] - avg;
            score += (int64_t)a * b;
        }

        score_avg = score / pair_count;
        if (score_avg > best_score_avg) {
            best_score_avg = score_avg;
            best_lag = lag;
        }
    }

    if (best_lag == 0U || best_score_avg <= 0) {
        return 0U;
    }

    return (uint32_t)(((uint64_t)sample_rate_hz * 10ULL + (best_lag / 2U)) / best_lag);
}

static void reset_frequency_display(void)
{
    g_ui.state.wave_freq_valid = false;
    g_ui.state.wave_freq_calc_valid = false;
    g_ui.state.wave_freq_x10 = 0U;
    g_ui.state.wave_freq_calc_x10 = 0U;
    g_ui.state.wave_freq_sum_x10 = 0U;
    g_ui.state.wave_freq_sample_count = 0U;
    g_ui.state.wave_freq_last_ms = 0U;
    g_ui.state.wave_low_freq_last_ms = 0U;
    g_ui.state.wave_period_avg_x100 = 0U;
    g_wave_low_freq_len = 0U;
}

/* newlib-nano has no floating printf enabled. Guard casts even if a future
 * calibration supplies very large (but finite) values outside display range.
 */
static void pm_format_fixed3(char *text, size_t size, float value)
{
    float magnitude = fabsf(value);
    if (!isfinite(value)) { snprintf(text, size, "--"); return; }
    if (magnitude >= 1000000.0f) { snprintf(text, size, "%s>=1e6", value < 0.0f ? "-" : ""); return; }
    uint32_t scaled = (uint32_t)(magnitude * 1000.0f + 0.5f);
    snprintf(text, size, "%s%lu.%03lu", value < 0.0f ? "-" : "",
             (unsigned long)(scaled / 1000U), (unsigned long)(scaled % 1000U));
}

static void update_pm_power(const pm_measurement_t *measurement)
{
    if (g_ui.label_pm_power == NULL || g_ui.label_pm_wavelength == NULL) { return; }
    char text[200];
    snprintf(text, sizeof(text), "Wavelength: %u nm", (unsigned int)measurement->wavelength_nm);
    lv_label_set_text(g_ui.label_pm_wavelength, text);

    const char *status;
    if (!measurement->valid) {
        status = measurement->adc_fault ? "ADC ERROR" :
                 measurement->range_switching ? "SWITCHING / WAIT NEW DATA" : "WAIT ADC";
    } else if (measurement->over_range) { status = "OVER RANGE";
    } else if (measurement->adc_mean >= (float)ADC_FULL_SCALE) { status = "ADC SATURATED";
    } else if (measurement->responsivity_A_W <= 0.0f) { status = "NO RESP DATA";
    } else if (!measurement->power_valid) { status = "NUMERIC ERROR";
    } else { status = measurement->out_of_spec ? "OUT OF SPEC" : "IN SPEC (NOMINAL)"; }

    char current[24];
    if (measurement->valid) { pm_format_fixed3(current, sizeof(current), measurement->photocurrent_A * 1000000.0f); }
    else { snprintf(current, sizeof(current), "--"); }
    if (measurement->power_valid) {
        char power[24], dbm[24];
        pm_format_fixed3(power, sizeof(power), measurement->power_mW);
        if (measurement->dbm_valid) { pm_format_fixed3(dbm, sizeof(dbm), measurement->power_dBm); }
        else { snprintf(dbm, sizeof(dbm), "-- (zero)"); }
        snprintf(text, sizeof(text), "Power: %s mW | %s dBm | %s\nIphoto: %s uA | Typical LUT, uncalibrated",
                 power, dbm, status, current);
    } else {
        snprintf(text, sizeof(text), "Power: -- mW | -- dBm | %s\nIphoto: %s uA | Typical LUT, uncalibrated",
                 status, current);
    }
    lv_label_set_text(g_ui.label_pm_power, text);
    lv_obj_set_style_text_color(g_ui.label_pm_power,
                               measurement->power_valid && !measurement->out_of_spec ? C_ACCENT : C_WARN, 0);
}

static void update_pm_debug(lv_timer_t *timer)
{
    (void)timer;
    if (g_ui.label_pm_debug == NULL) {
        return;
    }

    const pm_measurement_t *measurement = PM_GetMeasurement();
    update_pm_power(measurement);
    const char *mode = measurement->mode == PM_MODE_AUTO ? "AUTO" : "MANUAL";
    const char *range = "LOW";
    if (measurement->current_range == PM_RANGE_HIGH) { range = "HIGH"; }
    else if (measurement->current_range == PM_RANGE_MID) { range = "MID"; }
    char text[240];
    if (!measurement->valid) {
        const char *status = measurement->adc_fault ? "ADC ERROR" :
                             measurement->range_switching ? "SWITCHING / WAIT NEW DATA" : "WAIT ADC";
        snprintf(text, sizeof(text), "PM | %s | %s | %s\nADC RAW: --\nADC Voltage: --\nFiltered Voltage: --",
                 mode, range, status);
        lv_obj_set_style_text_color(g_ui.label_pm_debug, measurement->adc_fault ? C_BAD : C_WARN, 0);
        lv_label_set_text(g_ui.label_pm_debug, text);
        return;
    }

    /* Format with integer fields; the project uses newlib-nano without %f. */
    uint32_t voltage_mV = (uint32_t)(measurement->adc_voltage_V * 1000.0f + 0.5f);
    uint32_t filtered_mV = (uint32_t)(measurement->filtered_voltage_V * 1000.0f + 0.5f);
    uint32_t mean_x100 = (uint32_t)(measurement->adc_mean * 100.0f + 0.5f);
    snprintf(text, sizeof(text),
             "PM | %s | %s | %s\n"
             "ADC RAW: %u  Mean: %lu.%02lu  Min: %u  Max: %u\n"
             "ADC Voltage: %lu.%03lu V\n"
             "Filtered Voltage: %lu.%03lu V",
             mode, range, measurement->over_range ? "OVER RANGE" : "NORMAL",
             (unsigned int)measurement->adc_raw,
             (unsigned long)(mean_x100 / 100U), (unsigned long)(mean_x100 % 100U),
             (unsigned int)measurement->adc_min, (unsigned int)measurement->adc_max,
             (unsigned long)(voltage_mV / 1000U), (unsigned long)(voltage_mV % 1000U),
             (unsigned long)(filtered_mV / 1000U), (unsigned long)(filtered_mV % 1000U));
    lv_obj_set_style_text_color(g_ui.label_pm_debug, measurement->over_range ? C_BAD : C_ACCENT, 0);
    lv_label_set_text(g_ui.label_pm_debug, text);
}

static const char *pm_range_name(pm_range_t range)
{
    if (range == PM_RANGE_HIGH) { return "高"; }
    if (range == PM_RANGE_MID) { return "中"; }
    return "低";
}

static const char *pm_status_name(const pm_measurement_t *measurement)
{
    if (!measurement->valid) {
        return measurement->range_switching ? "切换中" :
               measurement->adc_fault ? "ADC错误" : "等待ADC";
    }
    if (measurement->over_range || measurement->adc_mean >= (float)ADC_FULL_SCALE) { return "过载"; }
    if (!measurement->power_valid) {
        return measurement->responsivity_A_W <= 0.0f ? "无响应数据" : "测量错误";
    }
    if (measurement->under_range) { return "过低"; }
    if (measurement->out_of_spec) { return "超出规格"; }
    return "正常";
}

static lv_color_t pm_status_color(const pm_measurement_t *measurement)
{
    if (measurement->valid && measurement->power_valid && !measurement->under_range &&
        !measurement->out_of_spec && !measurement->over_range) { return C_OK; }
    if (measurement->range_switching || measurement->under_range ||
        (measurement->valid && measurement->responsivity_A_W <= 0.0f)) { return C_WARN; }
    return C_BAD;
}

static void update_power_meter_ui(lv_timer_t *timer)
{
    (void)timer;
    if (g_ui.label_pm_main == NULL) { return; }

    const pm_measurement_t *measurement = PM_GetMeasurement();
    char text[96];
    char value[32];
    lv_color_t color = pm_status_color(measurement);
    if (!measurement->valid || !measurement->power_valid) {
        snprintf(text, sizeof(text), "--");
    } else if (g_ui.pm_show_dbm) {
        if (measurement->dbm_valid) {
            pm_format_fixed3(value, sizeof(value), measurement->power_dBm);
            snprintf(text, sizeof(text), "%sdBm", value);
        } else {
            snprintf(text, sizeof(text), "--dBm");
        }
    } else if (g_ui.pm_show_uW) {
        pm_format_fixed3(value, sizeof(value), measurement->power_uW);
        snprintf(text, sizeof(text), "%suW", value);
    } else {
        pm_format_fixed3(value, sizeof(value), measurement->power_mW);
        snprintf(text, sizeof(text), "%smW", value);
    }
    lv_label_set_text(g_ui.label_pm_main, text);
    lv_obj_set_style_text_color(g_ui.label_pm_main, color, 0);

    snprintf(text, sizeof(text), "波长:%unm", (unsigned int)measurement->wavelength_nm);
    lv_label_set_text(g_ui.label_pm_wavelength, text);
    snprintf(text, sizeof(text), "档位:%s%s",
             measurement->mode == PM_MODE_AUTO ? "自动" : "手动",
             pm_range_name(measurement->current_range));
    lv_label_set_text(g_ui.label_pm_range, text);
    pm_format_fixed3(value, sizeof(value), measurement->filtered_voltage_V);
    snprintf(text, sizeof(text), "前端:%sV", measurement->valid ? value : "--");
    lv_label_set_text(g_ui.label_pm_afe, text);
    snprintf(text, sizeof(text), "状态:%s", pm_status_name(measurement));
    lv_label_set_text(g_ui.label_pm_status, text);
    lv_obj_set_style_text_color(g_ui.label_pm_status, color, 0);
    pm_format_fixed3(value, sizeof(value), measurement->responsivity_A_W);
    char correction[24];
    pm_format_fixed3(correction, sizeof(correction), measurement->wavelength_correction);
    pm_wavelength_cal_point_t active_points[PM_WAVELENGTH_CAL_MAX_POINTS];
    uint8_t active_count=PM_GetWavelengthCalibrationPoints(active_points,PM_WAVELENGTH_CAL_MAX_POINTS);
    snprintf(text, sizeof(text), "响应:%sA/W校正:%s%s", value, correction,
             measurement->wavelength_calibrated ? "实测" : (active_count ? "插值" : "默认"));
    if(PM_GetZeroState()==PM_ZERO_IDLE) lv_label_set_text(g_ui.label_pm_hint,text);

    set_button_selected(g_ui.btn_pm_linear, !g_ui.pm_show_dbm);
    set_button_selected(g_ui.btn_pm_unit, !g_ui.pm_show_dbm && g_ui.pm_show_uW);
    set_button_selected(g_ui.btn_pm_auto, measurement->mode == PM_MODE_AUTO);
    for (uint32_t i = 0U; i < 3U; i++) {
        set_button_selected(g_ui.btn_pm_range[i], measurement->mode == PM_MODE_MANUAL &&
                            measurement->current_range == (pm_range_t)i);
    }
    pm_zero_ui_service();
    pm_cal_update_dialog();
}

static void pm_display_mode_event(lv_event_t *event)
{
    (void)event;
    g_ui.pm_show_dbm = !g_ui.pm_show_dbm;
    update_power_meter_ui(NULL);
}

static void pm_linear_unit_event(lv_event_t *event)
{
    (void)event;
    if (!g_ui.pm_show_dbm) { g_ui.pm_show_uW = !g_ui.pm_show_uW; }
    update_power_meter_ui(NULL);
}

static void pm_auto_range_event(lv_event_t *event)
{
    (void)event;
    const pm_measurement_t *measurement = PM_GetMeasurement();
    (void)PM_SetMode(measurement->mode == PM_MODE_AUTO ? PM_MODE_MANUAL : PM_MODE_AUTO);
    update_power_meter_ui(NULL);
}

#if 0 /* Legacy placeholder retained with the removed legacy SCOPE page. */
static void pm_reserved_event(lv_event_t *event)
{
    const char *name = (const char *)lv_event_get_user_data(event);
    if (g_ui.label_pm_hint != NULL) {
        lv_label_set_text(g_ui.label_pm_hint, name != NULL && name[0] == 'Z' ?
                          "清零功能待实现" :
                          "校准功能待实现");
    }
}

#endif

static void pm_power_wavelength_event(lv_event_t *event)
{
    static const int32_t steps[] = {-10, -1, 1, 10};
    uintptr_t index = (uintptr_t)lv_event_get_user_data(event);
    if (index < sizeof(steps) / sizeof(steps[0])) {
        (void)PM_SetWavelength((int32_t)PM_GetMeasurement()->wavelength_nm + steps[index]);
    }
    update_power_meter_ui(NULL);
}

static pm_display_mode_t pm_current_display_mode(void)
{
    if (g_ui.pm_show_dbm) { return PM_DISPLAY_MODE_DBM; }
    return g_ui.pm_show_uW ? PM_DISPLAY_MODE_UW : PM_DISPLAY_MODE_MW;
}

static void pm_set_display_mode(pm_display_mode_t mode)
{
    g_ui.pm_show_dbm = mode == PM_DISPLAY_MODE_DBM;
    g_ui.pm_show_uW = mode == PM_DISPLAY_MODE_UW;
}

static const char *pm_zero_state_name(pm_zero_state_t state)
{
    if (state == PM_ZERO_LOW) { return "清零低"; }
    if (state == PM_ZERO_MID) { return "清零中"; }
    if (state == PM_ZERO_HIGH) { return "清零高"; }
    if (state == PM_ZERO_RESTORING) { return "清零恢复中"; }
    return state == PM_ZERO_FAILED ? "清零失败" : "清零完成";
}

static void pm_zero_ui_service(void)
{
    pm_zero_state_t state = PM_GetZeroState();
    if (g_ui.label_pm_hint == NULL || state == PM_ZERO_IDLE) { return; }
    if (state == PM_ZERO_DONE && !g_ui.zero_save_handled)
    {
        g_ui.zero_save_handled = true;
        if (!PM_Storage_Save(pm_current_display_mode()))
        {
            (void)PM_SetPowerParameters(&g_ui.zero_backup);
            g_ui.zero_save_failed = true;
        }
    }
    lv_label_set_text(g_ui.label_pm_hint,
                      g_ui.zero_save_failed ? "清零失败" : pm_zero_state_name(state));
}

static void pm_zero_start(void)
{
    g_ui.zero_backup = *PM_GetPowerParameters();
    g_ui.zero_save_handled = false;
    g_ui.zero_save_failed = false;
    if (!PM_StartZero() && g_ui.label_pm_hint != NULL)
    {
        lv_label_set_text(g_ui.label_pm_hint, "清零失败");
    }
    pm_zero_ui_service();
    update_power_meter_ui(NULL);
}

static void pm_zero_confirm_event(lv_event_t *event)
{
    (void)event;
    if (g_ui.zero_confirm_overlay != NULL)
    {
        lv_obj_add_flag(g_ui.zero_confirm_overlay, LV_OBJ_FLAG_HIDDEN);
    }
    pm_zero_start();
}

static void pm_zero_cancel_event(lv_event_t *event)
{
    (void)event;
    if (g_ui.zero_confirm_overlay != NULL)
    {
        lv_obj_add_flag(g_ui.zero_confirm_overlay, LV_OBJ_FLAG_HIDDEN);
    }
}

static void pm_zero_event(lv_event_t *event)
{
    (void)event;
    pm_zero_state_t state = PM_GetZeroState();
    if (state == PM_ZERO_LOW || state == PM_ZERO_MID || state == PM_ZERO_HIGH ||
        state == PM_ZERO_RESTORING)
    {
        pm_zero_ui_service();
        return;
    }
    if (g_ui.zero_confirm_overlay != NULL)
    {
        lv_obj_clear_flag(g_ui.zero_confirm_overlay, LV_OBJ_FLAG_HIDDEN);
    }
}

static bool pm_parse_reference_mW(const char *input, float *reference_mW)
{
    uint32_t integer = 0U;
    uint32_t fraction = 0U;
    uint32_t divisor = 1U;
    bool decimal = false;
    bool digit = false;
    if (input == NULL || reference_mW == NULL) { return false; }
    for (size_t i = 0U; input[i] != '\0'; i++)
    {
        char c = input[i];
        if (c == '.')
        {
            if (decimal) { return false; }
            decimal = true;
            continue;
        }
        if (c < '0' || c > '9') { return false; }
        digit = true;
        uint32_t value = (uint32_t)(c - '0');
        if (!decimal)
        {
            if (integer > 100000U) { return false; }
            integer = integer * 10U + value;
        }
        else if (divisor < 1000U)
        {
            fraction = fraction * 10U + value;
            divisor *= 10U;
        }
        else { return false; }
    }
    float parsed = (float)integer + (float)fraction / (float)divisor;
    if (!digit || !isfinite(parsed) || parsed <= 0.0f) { return false; }
    *reference_mW = parsed;
    return true;
}

static void pm_cal_update_dialog(void)
{
    if (g_ui.cal_overlay == NULL) { return; }
    const pm_measurement_t *measurement = PM_GetMeasurement();
    const pm_power_parameters_t *parameters = PM_GetPowerParameters();
    char text[48];
    snprintf(text, sizeof(text), "参考功率:%smW", g_ui.cal_input[0] ? g_ui.cal_input : "0");
    lv_label_set_text(g_ui.label_cal_input, text);
    snprintf(text, sizeof(text), "波长:%unm", (unsigned int)measurement->wavelength_nm);
    lv_label_set_text(g_ui.label_cal_wave, text);

    char value[24];
    pm_format_fixed3(value, sizeof(value), measurement->filtered_voltage_V);
    snprintf(text, sizeof(text), "当前AFE:%sV", measurement->valid ? value : "--");
    lv_label_set_text(g_ui.label_cal_afe, text);
    pm_format_fixed3(value, sizeof(value), measurement->raw_power_mW);
    snprintf(text, sizeof(text), "功率(未校正):%smW", measurement->raw_power_valid ? value : "--");
    lv_label_set_text(g_ui.label_cal_raw_power, text);
    snprintf(text, sizeof(text), "当前档位:%s", pm_range_name(measurement->current_range));
    lv_label_set_text(g_ui.label_cal_current_range, text);
    pm_format_fixed3(value, sizeof(value), parameters->gain_correction[g_ui.cal_range]);
    snprintf(text, sizeof(text), "当前校正:%s", value);
    lv_label_set_text(g_ui.label_cal_gain, text);
    pm_wavelength_cal_point_t points[PM_WAVELENGTH_CAL_MAX_POINTS];
    uint8_t count=PM_GetWavelengthCalibrationPoints(points,PM_WAVELENGTH_CAL_MAX_POINTS);
    size_t used=(size_t)snprintf(text,sizeof(text),"点:");
    for(uint8_t i=0U;i<count&&used<sizeof(text);i++){char factor[16];pm_format_fixed3(factor,sizeof(factor),points[i].correction_factor);used+=(size_t)snprintf(&text[used],sizeof(text)-used,"%u:%s",(unsigned)points[i].wavelength_nm,factor);}
    if(count==0U) snprintf(text,sizeof(text),"点:0");
    lv_label_set_text(g_ui.label_cal_points,text);
    for (uint32_t i = 0U; i < 3U; i++) {
        set_button_selected(g_ui.btn_cal_range[i], g_ui.cal_range == (pm_range_t)i);
    }
}

static const char *pm_calibration_check_name(pm_calibration_check_t check)
{
    switch (check)
    {
        case PM_CAL_CHECK_OK: return "校准";
        case PM_CAL_CHECK_REFERENCE_INVALID: return "参考功率错误";
        case PM_CAL_CHECK_ADC_INVALID: return "ADC错误";
        case PM_CAL_CHECK_RANGE_SWITCHING: return "等待档位稳定";
        case PM_CAL_CHECK_RANGE_MISMATCH: return "切换校准档";
        case PM_CAL_CHECK_OVER_RANGE: return "过载";
        case PM_CAL_CHECK_RESPONSIVITY_INVALID: return "无响应数据";
        case PM_CAL_CHECK_SIGNAL_TOO_LOW: return "数据过低";
        case PM_CAL_CHECK_RAW_POWER_INVALID: return "功率错误";
        default: return "校准失败";
    }
}

static void pm_cal_range_event(lv_event_t *event)
{
    g_ui.cal_range = (pm_range_t)(uintptr_t)lv_event_get_user_data(event);
    (void)PM_SetManualRange(g_ui.cal_range);
    pm_cal_update_dialog();
    update_power_meter_ui(NULL);
}

static void pm_cal_wavelength_event(lv_event_t *event)
{
    static const int32_t steps[] = {-10, -1, 1, 10};
    uintptr_t index = (uintptr_t)lv_event_get_user_data(event);
    if (index < sizeof(steps) / sizeof(steps[0]))
    {
        (void)PM_SetWavelength((int32_t)PM_GetMeasurement()->wavelength_nm + steps[index]);
    }
    pm_cal_update_dialog();
    update_power_meter_ui(NULL);
}

static void pm_cal_key_event(lv_event_t *event)
{
    const char *key = (const char *)lv_event_get_user_data(event);
    size_t length = strlen(g_ui.cal_input);
    if (key == NULL) { return; }
    if (strcmp(key, "DEL") == 0)
    {
        if (length > 0U) { g_ui.cal_input[length - 1U] = '\0'; }
    }
    else if (strcmp(key, "DOT") == 0)
    {
        if (length == 0U) { snprintf(g_ui.cal_input, sizeof(g_ui.cal_input), "0."); }
        else if (strchr(g_ui.cal_input, '.') == NULL && length + 1U < sizeof(g_ui.cal_input))
        {
            g_ui.cal_input[length] = '.';
            g_ui.cal_input[length + 1U] = '\0';
        }
    }
    else if (key[0] >= '0' && key[0] <= '9' && key[1] == '\0' && length + 1U < sizeof(g_ui.cal_input))
    {
        g_ui.cal_input[length] = key[0];
        g_ui.cal_input[length + 1U] = '\0';
    }
    pm_cal_update_dialog();
}

static void pm_cal_cancel_event(lv_event_t *event)
{
    (void)event;
    if (g_ui.cal_overlay != NULL) { lv_obj_add_flag(g_ui.cal_overlay, LV_OBJ_FLAG_HIDDEN); }
}

static void pm_cal_confirm_event(lv_event_t *event)
{
    (void)event;
    float reference_mW;
    if (!pm_parse_reference_mW(g_ui.cal_input, &reference_mW))
    {
        lv_label_set_text(g_ui.label_cal_status, "参考功率错误");
        return;
    }
    pm_calibration_check_t check = PM_CheckCalibration(g_ui.cal_range, reference_mW);
    if (check != PM_CAL_CHECK_OK)
    {
        lv_label_set_text(g_ui.label_cal_status, pm_calibration_check_name(check));
        return;
    }
    g_ui.cal_backup = *PM_GetPowerParameters();
    if (!PM_CalibrateRange(g_ui.cal_range, reference_mW))
    {
        lv_label_set_text(g_ui.label_cal_status, "校准失败");
        return;
    }
    if (!PM_Storage_Save(pm_current_display_mode()))
    {
        (void)PM_SetPowerParameters(&g_ui.cal_backup);
        lv_label_set_text(g_ui.label_cal_status, "校准失败");
        return;
    }
    if (g_ui.label_pm_hint != NULL) { lv_label_set_text(g_ui.label_pm_hint, "校准完成"); }
    lv_obj_add_flag(g_ui.cal_overlay, LV_OBJ_FLAG_HIDDEN);
    update_power_meter_ui(NULL);
}

static void pm_wavelength_cal_event(lv_event_t *event)
{
    (void)event; float reference_mW;
    if(!pm_parse_reference_mW(g_ui.cal_input,&reference_mW)){lv_label_set_text(g_ui.label_cal_status,"参考功率错误");return;}
    pm_calibration_check_t check=PM_CheckCalibration(PM_GetMeasurement()->current_range,reference_mW);
    if(check!=PM_CAL_CHECK_OK){lv_label_set_text(g_ui.label_cal_status,pm_calibration_check_name(check));return;}
    if(!PM_CalibrateWavelength(reference_mW)||!PM_Storage_Save(pm_current_display_mode())){lv_label_set_text(g_ui.label_cal_status,"校准失败");return;}
    lv_label_set_text(g_ui.label_cal_status,"波长校准完成");
}
static void pm_wavelength_delete_event(lv_event_t *event)
{(void)event; if(PM_DeleteWavelengthCalibrationPoint(PM_GetMeasurement()->wavelength_nm)&&PM_Storage_Save(pm_current_display_mode()))lv_label_set_text(g_ui.label_cal_status,"删除完成");else lv_label_set_text(g_ui.label_cal_status,"删除失败");}
static void pm_wavelength_reset_event(lv_event_t *event)
{(void)event; PM_ResetWavelengthCalibration(); if(PM_Storage_Save(pm_current_display_mode()))lv_label_set_text(g_ui.label_cal_status,"恢复完成");else lv_label_set_text(g_ui.label_cal_status,"恢复失败");}

static void pm_cal_open_event(lv_event_t *event)
{
    (void)event;
    if (PM_GetZeroState() == PM_ZERO_LOW || PM_GetZeroState() == PM_ZERO_MID ||
        PM_GetZeroState() == PM_ZERO_HIGH || PM_GetZeroState() == PM_ZERO_RESTORING)
    {
        if (g_ui.label_pm_hint != NULL) { lv_label_set_text(g_ui.label_pm_hint, "清零中"); }
        return;
    }
    g_ui.cal_range = PM_GetMeasurement()->current_range;
    g_ui.cal_input[0] = '\0';
    lv_label_set_text(g_ui.label_cal_status, "");
    lv_obj_clear_flag(g_ui.cal_overlay, LV_OBJ_FLAG_HIDDEN);
    pm_cal_update_dialog();
}

static void pm_save_settings_event(lv_event_t *event)
{
    (void)event;
    if (g_ui.label_pm_hint != NULL)
    {
        lv_label_set_text(g_ui.label_pm_hint,
                          PM_Storage_Save(pm_current_display_mode()) ? "保存完成" : "保存失败");
    }
}

static void render_zero_confirm_dialog(lv_obj_t *parent, int32_t width, int32_t height)
{
    g_ui.zero_confirm_overlay = make_obj(parent, 0, 0, width, height, C_BG);
    lv_obj_t *dialog = make_panel(g_ui.zero_confirm_overlay, (width - 380) / 2, 128, 380, 190, "清零");
    make_label(dialog, "无光S1223-01", 26, 56, &lv_font_chinese_14, C_TEXT);
    make_label(dialog, "确认", 26, 88, &lv_font_chinese_14, C_MUTED);
    make_button(dialog, "取消", 26, 132, 130, 34, C_PANEL_2, pm_zero_cancel_event);
    make_button(dialog, "确认", 204, 132, 130, 34, C_ACCENT, pm_zero_confirm_event);
    lv_obj_add_flag(g_ui.zero_confirm_overlay, LV_OBJ_FLAG_HIDDEN);
}

static void render_calibration_dialog(lv_obj_t *parent, int32_t width, int32_t height)
{
    g_ui.cal_overlay = make_obj(parent, 0, 0, width, height, C_BG);
    lv_obj_t *dialog = make_panel(g_ui.cal_overlay, 10, 54, width - 20, height - 64, "校准");
    g_ui.label_cal_afe = make_label(dialog, "当前AFE:--V", 14, 42, &lv_font_chinese_14, C_TEXT);
    g_ui.label_cal_raw_power = make_label(dialog, "功率(未校正):--mW", 14, 68, &lv_font_chinese_14, C_TEXT);
    g_ui.label_cal_current_range = make_label(dialog, "当前档位:--", 14, 94, &lv_font_chinese_14, C_TEXT);
    g_ui.label_cal_gain = make_label(dialog, "当前校正:--", 14, 120, &lv_font_chinese_14, C_TEXT);

    make_label(dialog, "校准档位", 300, 42, &lv_font_chinese_14, C_TEXT);
    static const char *range_names[] = {"高", "中", "低"};
    for (uint32_t i = 0U; i < 3U; i++) {
        g_ui.btn_cal_range[i] = make_button(dialog, range_names[i], 398 + (int32_t)i * 62, 34,
                                             54, 30, C_PANEL_2, NULL);
        lv_obj_add_event_cb(g_ui.btn_cal_range[i], pm_cal_range_event, LV_EVENT_CLICKED, (void *)(uintptr_t)i);
    }
    g_ui.label_cal_wave = make_label(dialog, "波长:--", 300, 78, &lv_font_chinese_14, C_MUTED);
    static const char *wave_steps[] = {"-10", "-1", "+1", "+10"};
    for (uint32_t i = 0U; i < 4U; i++) {
        lv_obj_t *button = make_button(dialog, wave_steps[i], 398 + (int32_t)i * 62, 70,
                                       54, 30, C_PANEL_2, NULL);
        lv_obj_add_event_cb(button, pm_cal_wavelength_event, LV_EVENT_CLICKED, (void *)(uintptr_t)i);
    }
    g_ui.label_cal_input = make_label(dialog, "参考功率:0mW", 300, 112, &lv_font_chinese_14, C_TEXT);
    g_ui.label_cal_status = make_label(dialog, "", 300, 138, &lv_font_chinese_14, C_WARN);
    lv_obj_set_width(g_ui.label_cal_status, width - 330);

    static const char *keys[] = {"1", "2", "3", "4", "5", "6", "7", "8", "9", "DOT", "0", "DEL"};
    static const char *key_text[] = {"1", "2", "3", "4", "5", "6", "7", "8", "9", ".", "0", "删除"};
    for (uint32_t i = 0U; i < 12U; i++) {
        int32_t x = 14 + (int32_t)(i % 3U) * 70;
        int32_t y = 158 + (int32_t)(i / 3U) * 42;
        lv_obj_t *button = make_button(dialog, key_text[i], x, y, 60, 34, C_PANEL_2, NULL);
        lv_obj_add_event_cb(button, pm_cal_key_event, LV_EVENT_CLICKED, (void *)keys[i]);
    }
    make_button(dialog, "取消", 300, 180, 120, 34, C_PANEL_2, pm_cal_cancel_event);
    make_button(dialog, "校准", 438, 180, 120, 34, C_ACCENT, pm_cal_confirm_event);
    make_button(dialog, "波长校准", 300, 230, 120, 34, C_PANEL_2, pm_wavelength_cal_event);
    make_button(dialog, "删除", 438, 230, 58, 34, C_PANEL_2, pm_wavelength_delete_event);
    make_button(dialog, "恢复", 510, 230, 58, 34, C_PANEL_2, pm_wavelength_reset_event);
    g_ui.label_cal_points=make_label(dialog,"点:0",300,280,&lv_font_chinese_14,C_MUTED);
    lv_obj_add_flag(g_ui.cal_overlay, LV_OBJ_FLAG_HIDDEN);
}

static void render_power_meter(void)
{
    lv_obj_t *c = g_ui.content;
    int32_t width = lv_obj_get_width(c);
    int32_t height = lv_obj_get_height(c);
    if (width <= 0) { width = 800; }
    if (height <= 0) { height = 480; }
    int32_t pad = 10;
    int32_t right_w = 260;
    int32_t main_w = width - right_w - pad * 3;
    if (main_w < 360) { main_w = 360; right_w = width - main_w - pad * 3; }

    lv_obj_t *header = make_obj(c, 0, 0, width, 44, C_BG);
    make_label(header, "光功率计", 14, 13, &lv_font_chinese_14, C_TEXT);
    make_label(header, "STM32H743|S1223-01", width - 190, 14, &lv_font_chinese_14, C_MUTED);
    make_obj(c, 0, 43, width, 1, C_LINE);

    lv_obj_t *reading = make_panel(c, pad, 54, main_w, 130, "已测光功率");
    g_ui.label_pm_main = make_label(reading, "--", 16, 50, &lv_font_montserrat_32, C_WARN);
    lv_obj_set_width(g_ui.label_pm_main, main_w - 32);
    lv_label_set_long_mode(g_ui.label_pm_main, LV_LABEL_LONG_CLIP);

    lv_obj_t *info = make_panel(c, pad * 2 + main_w, 54, right_w, 130, "实时状态");
    g_ui.label_pm_wavelength = make_label(info, "波长:--", 14, 42, &lv_font_chinese_14, C_TEXT);
    g_ui.label_pm_range = make_label(info, "档位:--", 14, 64, &lv_font_chinese_14, C_TEXT);
    g_ui.label_pm_afe = make_label(info, "前端:--V", 14, 86, &lv_font_chinese_14, C_TEXT);
    g_ui.label_pm_status = make_label(info, "状态:等待ADC", 14, 108, &lv_font_chinese_14, C_WARN);

    lv_obj_t *display = make_panel(c, pad, 194, main_w, 62, "显示");
    g_ui.btn_pm_linear = make_button(display, "线性/dBm", 14, 27, 144, 30, C_PANEL_2, pm_display_mode_event);
    g_ui.btn_pm_unit = make_button(display, "mW/uW", 170, 27, 108, 30, C_PANEL_2, pm_linear_unit_event);
    make_label(display, "线性支持mW和uW", 290, 35, &lv_font_chinese_14, C_MUTED);

    lv_obj_t *range = make_panel(c, pad * 2 + main_w, 194, right_w, 62, "量程控制");
    g_ui.btn_pm_auto = make_button(range, "自动量程", 12, 27, 112, 30, C_PANEL_2, pm_auto_range_event);
    static const char *range_names[] = {"高", "中", "低"};
    for (uint32_t i = 0U; i < 3U; i++) {
        g_ui.btn_pm_range[i] = make_button(range, range_names[i], 132 + (int32_t)i * 40, 27,
                                            36, 30, C_PANEL_2, NULL);
        lv_obj_add_event_cb(g_ui.btn_pm_range[i], pm_manual_event, LV_EVENT_CLICKED, (void *)(uintptr_t)i);
    }

    lv_obj_t *wave = make_panel(c, pad, 266, width - pad * 2, 62, "波长设置(400-1000nm)");
    static const char *wave_steps[] = {"-10", "-1", "+1", "+10"};
    for (uint32_t i = 0U; i < 4U; i++) {
        lv_obj_t *button = make_button(wave, wave_steps[i], 14 + (int32_t)i * 64, 27, 56, 30, C_PANEL_2, NULL);
        lv_obj_add_event_cb(button, pm_power_wavelength_event, LV_EVENT_CLICKED, (void *)(uintptr_t)i);
    }
    g_ui.label_pm_hint = make_label(wave, "响应表:660-960nm", 292, 35,
                                    &lv_font_chinese_14, C_MUTED);

    lv_obj_t *actions = make_panel(c, pad, 338, width - pad * 2, 58, "校准");
    lv_obj_t *zero = make_button(actions, "清零", 14, 25, 100, 30, C_PANEL_2, NULL);
    lv_obj_add_event_cb(zero, pm_zero_event, LV_EVENT_CLICKED, NULL);
    lv_obj_t *cal = make_button(actions, "校准", 126, 25, 100, 30, C_PANEL_2, NULL);
    lv_obj_add_event_cb(cal, pm_cal_open_event, LV_EVENT_CLICKED, NULL);
    make_label(actions, "当前阶段不修改零点或校准值",
               242, 33, &lv_font_chinese_14, C_MUTED);

    lv_obj_t *save_area = make_obj(actions, 238, 25, width - 262, 30, C_PANEL);
    make_button(save_area, "保存设置", 0, 0, 100, 30, C_PANEL_2, pm_save_settings_event);
    make_label(save_area, "清零和校准完成后自动保存", 112, 8, &lv_font_chinese_14, C_MUTED);
    render_zero_confirm_dialog(c, width, height);
    render_calibration_dialog(c, width, height);
    update_power_meter_ui(NULL);
}

static void pm_auto_event(lv_event_t *event)
{
    (void)event;
    (void)PM_SetMode(PM_MODE_AUTO);
    update_pm_debug(NULL);
}

static void pm_manual_event(lv_event_t *event)
{
    pm_range_t range = (pm_range_t)(uintptr_t)lv_event_get_user_data(event);
    (void)PM_SetManualRange(range);
    update_pm_debug(NULL);
    update_power_meter_ui(NULL);
}

static void pm_wavelength_event(lv_event_t *event)
{
    static const int32_t steps[] = {-10, -1, 1, 10};
    uintptr_t index = (uintptr_t)lv_event_get_user_data(event);
    if (index >= sizeof(steps) / sizeof(steps[0])) { return; }
    (void)PM_SetWavelength((int32_t)PM_GetMeasurement()->wavelength_nm + steps[index]);
    update_pm_debug(NULL);
    update_power_meter_ui(NULL);
}

static void render_scope(void)
{
    lv_obj_t *c = g_ui.content;

    lv_obj_t *scope = make_panel(c, 8, 8, 784, 336, "ADC Waveform - PA5 / ADC1 CH19");
    make_label(scope, "ADC and AFSK receiver are running. Wave drawing is paused to reduce LVGL load.", 14, 42,
               &lv_font_montserrat_14, C_WARN);

#if WAVE_DRAW_ENABLE
    lv_obj_t *plot = make_panel(scope, 14, 70, 756, 210, NULL);
    lv_obj_set_style_bg_color(plot, lv_color_hex(0x101820), 0);

    for (int32_t i = 1; i < 4; i++) {
        make_obj(plot, (756 * i) / 4, 0, 1, 210, C_LINE);
    }
    for (int32_t i = 1; i < 4; i++) {
        make_obj(plot, 0, (210 * i) / 4, 756, 1, C_LINE);
    }
    make_obj(plot, 0, 105, 756, 1, lv_color_hex(0x546575));

    g_ui.wave_line = lv_line_create(plot);
    lv_obj_set_style_line_color(g_ui.wave_line, C_ACCENT, 0);
    lv_obj_set_style_line_width(g_ui.wave_line, 2, 0);
    lv_obj_set_style_line_rounded(g_ui.wave_line, false, 0);
    lv_obj_set_pos(g_ui.wave_line, 0, 0);

    g_ui.label_wave_info = make_label(scope, "", 14, 292, &lv_font_montserrat_14, C_TEXT);
#else
    g_ui.wave_line = NULL;
    g_ui.label_pm_power = make_label(scope, "", 14, 76, &lv_font_montserrat_14, C_ACCENT);
    lv_obj_set_width(g_ui.label_pm_power, 740);
    lv_label_set_long_mode(g_ui.label_pm_power, LV_LABEL_LONG_CLIP);
    g_ui.label_wave_info = make_label(scope, "", 14, 126, &lv_font_montserrat_14, C_TEXT);
    g_ui.label_pm_wavelength = make_label(scope, "", 14, 155, &lv_font_montserrat_14, C_TEXT);
    static const char *wavelength_steps[] = {"-10", "-1", "+1", "+10"};
    for (uint32_t i = 0U; i < 4U; i++) {
        lv_obj_t *button = make_button(scope, wavelength_steps[i], 194 + (int32_t)i * 66, 150,
                                       60, 28, C_PANEL_2, NULL);
        lv_obj_add_event_cb(button, pm_wavelength_event, LV_EVENT_CLICKED, (void *)(uintptr_t)i);
    }
    make_label(scope, "LUT: 660-960 nm (typ.)", 476, 155, &lv_font_montserrat_14, C_MUTED);
    g_ui.label_pm_debug = make_label(scope, "", 14, 184, &lv_font_montserrat_14, C_ACCENT);
    lv_obj_set_width(g_ui.label_pm_debug, 740);
    lv_label_set_long_mode(g_ui.label_pm_debug, LV_LABEL_LONG_WRAP);
    update_pm_debug(NULL);
    make_button(scope, "AUTO", 14, 274, 100, 32, C_PANEL_2, pm_auto_event);
    static const char *range_names[] = {"HIGH", "MID", "LOW"};
    for (uint32_t i = 0U; i < 3U; i++) {
        lv_obj_t *button = make_button(scope, range_names[i], 126 + (int32_t)i * 112, 274,
                                       100, 32, C_PANEL_2, NULL);
        lv_obj_add_event_cb(button, pm_manual_event, LV_EVENT_CLICKED, (void *)(uintptr_t)i);
    }
#endif
    lv_obj_set_width(g_ui.label_wave_info, 740);
    lv_label_set_long_mode(g_ui.label_wave_info, LV_LABEL_LONG_CLIP);

    make_label(scope, "AFSK: 1200/2200Hz, 100bps, 16-phase bit-window search.", 14, 314,
               &lv_font_montserrat_14, C_MUTED);
    update_waveform_obj();
}

static void render_page(slave_page_t page)
{
    if (page >= PAGE_COUNT) {
        return;
    }

    g_ui.page = page;
    g_ui.label_station_value = NULL;
    g_ui.label_group_value = NULL;
    g_ui.label_wave_info = NULL;
    g_ui.label_pm_debug = NULL;
    g_ui.label_pm_power = NULL;
    g_ui.label_pm_wavelength = NULL;
    g_ui.label_pm_main = NULL;
    g_ui.label_pm_range = NULL;
    g_ui.label_pm_afe = NULL;
    g_ui.label_pm_status = NULL;
    g_ui.label_pm_hint = NULL;
    g_ui.btn_pm_linear = NULL;
    g_ui.btn_pm_unit = NULL;
    g_ui.btn_pm_auto = NULL;
    memset(g_ui.btn_pm_range, 0, sizeof(g_ui.btn_pm_range));
    g_ui.zero_confirm_overlay = NULL;
    g_ui.cal_overlay = NULL;
    g_ui.label_cal_input = NULL;
    g_ui.label_cal_wave = NULL;
    g_ui.label_cal_status = NULL;
    g_ui.label_cal_afe = NULL;
    g_ui.label_cal_raw_power = NULL;
    g_ui.label_cal_current_range = NULL;
    g_ui.label_cal_gain = NULL;
    g_ui.label_cal_points = NULL;
    memset(g_ui.btn_cal_range, 0, sizeof(g_ui.btn_cal_range));
    g_ui.wave_line = NULL;
    g_ui.bar_af = NULL;
    g_ui.bar_rssi = NULL;
    lv_obj_clean(g_ui.content);

    switch (page) {
    case PAGE_POWER:
        render_power_meter();
        break;
    case PAGE_HOME:
        render_home();
        break;
    case PAGE_RX:
        render_rx();
        break;
    case PAGE_SMS:
        render_sms();
        break;
    case PAGE_SETUP:
        render_setup();
        break;
    case PAGE_SCOPE:
        render_scope();
        break;
    default:
        break;
    }

    update_nav();
    update_top();
}

static void timer_event(lv_timer_t *timer)
{
    (void)timer;
    char buf[48];
    uint32_t sec = HAL_GetTick() / 1000U;
    uint32_t h = sec / 3600U;
    uint32_t m = (sec % 3600U) / 60U;
    uint32_t s = sec % 60U;

    if (g_ui.label_uptime != NULL) {
        snprintf(buf, sizeof(buf), "%02lu:%02lu:%02lu",
                 (unsigned long)h,
                 (unsigned long)m,
                 (unsigned long)s);
        lv_label_set_text(g_ui.label_uptime, buf);
    }

    if (g_ui.label_status != NULL) {
        snprintf(buf, sizeof(buf), "RSSI %ddBm  AF %u%%",
                 (int)g_ui.state.rssi_dbm,
                 g_ui.state.af_level);
        lv_label_set_text(g_ui.label_status, buf);
    }

    update_top();
}

void slave_ui_create(lv_obj_t *parent)
{
    int32_t sw;
    int32_t sh;

    memset(&g_ui, 0, sizeof(g_ui));
    g_ui.root = parent;
    g_ui.state.station_id = 3U;
    g_ui.state.group_enabled = true;
    g_ui.state.rssi_dbm = -96;
    g_ui.state.af_level = 0U;
    g_ui.state.volume = 65U;
    g_ui.state.squelch = 35U;
    g_ui.state.battery_mv = 7400U;
    g_ui.state.battery_percent = 84U;
    g_ui.state.last_sender = 0U;
    snprintf(g_ui.state.pynq_status, sizeof(g_ui.state.pynq_status), "BOOT WAIT");
    snprintf(g_ui.state.capture_state, sizeof(g_ui.state.capture_state), "IDLE");
    snprintf(g_ui.state.test_result, sizeof(g_ui.state.test_result), "--");
    g_ui.state.wave_min = 0U;
    g_ui.state.wave_max = ADC_FULL_SCALE;
    g_ui.state.wave_avg = ADC_FULL_SCALE / 2U;
    g_ui.state.wave_pp_mv = ADC_REF_MV;
    g_ui.state.wave_abs_sample = 0U;
    g_ui.state.wave_last_cross_x100 = 0U;
    g_ui.state.wave_above_high = false;
    pm_set_display_mode(g_pm_start_display_mode);
    reset_frequency_display();

    for (uint16_t i = 0; i < WAVE_POINT_COUNT; i++) {
        g_ui.wave_points[i].x = (int32_t)((i * 756U) / (WAVE_POINT_COUNT - 1U));
        g_ui.wave_points[i].y = 105;
    }

    lv_obj_clean(parent);
    lv_obj_clear_flag(parent, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(parent, C_BG, 0);
    lv_obj_set_style_pad_all(parent, 0, 0);

    sw = lv_obj_get_width(parent);
    sh = lv_obj_get_height(parent);
    if (sw <= 0) {
        sw = 800;
    }
    if (sh <= 0) {
        sh = 480;
    }

    /* The legacy radio pages above remain in the binary for reference, but this
     * product exposes only the power-meter screen: no hidden navigation layer. */
    g_ui.content = make_obj(parent, 0, 0, sw, sh, C_BG);

    lv_timer_create(timer_event, 250, NULL);
    lv_timer_create(update_power_meter_ui, 100, NULL);
    render_page(PAGE_POWER);
}

void slave_ui_set_power_meter_display_mode(uint8_t display_mode)
{
    if (display_mode > PM_DISPLAY_MODE_DBM) { return; }
    g_pm_start_display_mode = (pm_display_mode_t)display_mode;
    pm_set_display_mode(g_pm_start_display_mode);
    update_power_meter_ui(NULL);
}

void slave_ui_set_command_callback(void (*callback)(const char *command))
{
    g_command_callback = callback;
}

void slave_ui_set_pynq_status(const char *status)
{
    if (status == NULL) {
        status = "";
    }
    snprintf(g_ui.state.pynq_status, sizeof(g_ui.state.pynq_status), "%s", status);
    update_pynq_labels();
}

void slave_ui_set_test_result(const char *result)
{
    if (result == NULL) {
        result = "";
    }
    snprintf(g_ui.state.test_result, sizeof(g_ui.state.test_result), "%s", result);
    update_pynq_labels();
}

void slave_ui_set_capture_state(const char *state)
{
    if (state == NULL) {
        state = "";
    }
    snprintf(g_ui.state.capture_state, sizeof(g_ui.state.capture_state), "%s", state);
    update_pynq_labels();
}

void slave_ui_append_log(const char *line)
{
    if (line == NULL || line[0] == '\0') {
        return;
    }

    if (g_ui.state.log_count < PYNQ_LOG_LINES) {
        g_ui.state.log_count++;
    } else {
        memmove(g_ui.state.log_lines,
                &g_ui.state.log_lines[1],
                (PYNQ_LOG_LINES - 1U) * PYNQ_LOG_LINE_MAX);
    }

    snprintf(g_ui.state.log_lines[g_ui.state.log_count - 1U],
             PYNQ_LOG_LINE_MAX,
             "%s",
             line);
    update_pynq_labels();
}

void slave_ui_set_rx_state(bool carrier_detected,
                           bool selected_call,
                           bool group_call,
                           int16_t rssi_dbm,
                           uint8_t af_level)
{
    g_ui.state.carrier_detected = carrier_detected;
    g_ui.state.selected_call = selected_call;
    g_ui.state.group_call = group_call;
    g_ui.state.rssi_dbm = rssi_dbm;
    g_ui.state.af_level = af_level > 100U ? 100U : af_level;
    update_top();
}

void slave_ui_set_sms(const char *text, uint8_t sender_id, bool group_call)
{
    if (text == NULL) {
        text = "";
    }

    /* ===== Address filter =====
     * Group call (sender_id == 0xFF): ALWAYS accept, regardless of group_enabled.
     * Normal call: only accept if sender_id matches local station_id.
     */
    if (sender_id != 0xFFU && sender_id != g_ui.state.station_id) {
        char drop_msg[96];
        snprintf(drop_msg, sizeof(drop_msg), "DROP S%u: %s", sender_id,
                 text[0] ? text : "(empty)");
        slave_ui_append_log(drop_msg);
        return;
    }
    /* ===== End address filter ===== */

    snprintf(g_ui.state.sms_text, sizeof(g_ui.state.sms_text), "%s", text);
    g_ui.state.last_sender = (sender_id == 0xFFU) ? 0U : (sender_id % SLAVE_STATION_COUNT);
    g_ui.state.group_call = group_call;
    g_ui.state.packet_count++;
    g_ui.state.carrier_detected = true;
    g_ui.state.selected_call = true;

    /* Visible feedback: update capture_state so user knows SMS was received */
    snprintf(g_ui.state.capture_state, sizeof(g_ui.state.capture_state),
             "SMS rcvd S%u", sender_id == 0xFFU ? 0U : sender_id);
    update_top();

    /* Refresh current page so SMS appears immediately on any page */
    render_page(g_ui.page);
}

void slave_ui_set_battery(uint16_t millivolts, uint8_t percent)
{
    g_ui.state.battery_mv = millivolts;
    g_ui.state.battery_percent = percent > 100U ? 100U : percent;
    update_top();
}

void slave_ui_reset_waveform(void)
{
    reset_frequency_display();
    g_ui.state.wave_abs_sample = 0U;
    g_ui.state.wave_last_cross_x100 = 0U;
    g_ui.state.wave_above_high = false;
}

void slave_ui_set_waveform(const uint16_t *samples, uint16_t count)
{
    uint32_t sum = 0;
    uint16_t min_raw = ADC_FULL_SCALE;
    uint16_t max_raw = 0;
    uint32_t avg_raw;

    if (samples == NULL || count == 0U) {
        return;
    }

    for (uint16_t i = 0; i < count; i++) {
        uint16_t v = samples[i];
        if (v < min_raw) {
            min_raw = v;
        }
        if (v > max_raw) {
            max_raw = v;
        }
        sum += v;
    }

    avg_raw = sum / count;
    g_ui.state.wave_min = min_raw;
    g_ui.state.wave_max = max_raw;
    g_ui.state.wave_avg = (uint16_t)avg_raw;
    g_ui.state.wave_pp_mv = raw_to_mv(max_raw - min_raw);
    g_ui.state.wave_frame_count++;
    append_low_frequency_samples(samples, count);

#if WAVE_DRAW_ENABLE
    uint32_t span = (uint32_t)max_raw - min_raw;
    if (span < 64U) {
        span = 64U;
    }

    for (uint16_t i = 0; i < WAVE_POINT_COUNT; i++) {
        uint32_t src_index = ((uint32_t)i * (count - 1U)) / (WAVE_POINT_COUNT - 1U);
        uint16_t raw = samples[src_index];
        int32_t y = 204 - (int32_t)(((uint32_t)(raw - min_raw) * 198U) / span);

        if (y < 6) {
            y = 6;
        } else if (y > 204) {
            y = 204;
        }

        g_ui.wave_points[i].x = (int32_t)((i * 756U) / (WAVE_POINT_COUNT - 1U));
        g_ui.wave_points[i].y = y;
    }
#endif

    if (((uint32_t)max_raw - min_raw) >= WAVE_MIN_SPAN_RAW) {
        int32_t hyst = (int32_t)(((uint32_t)max_raw - min_raw) / 8U);
        int32_t high_thr = (int32_t)avg_raw + hyst;
        int32_t low_thr = (int32_t)avg_raw - hyst;

        for (uint16_t i = 1; i < count; i++) {
            int32_t prev = samples[i - 1U];
            int32_t cur = samples[i];

            if (g_ui.state.wave_above_high) {
                if (cur < low_thr) {
                    g_ui.state.wave_above_high = false;
                }
            } else if (cur >= high_thr) {
                uint32_t local_cross_x100;
                uint64_t abs_cross_x100;
                uint32_t den = (uint32_t)(cur - prev);

                if (den == 0U || prev >= (int32_t)avg_raw) {
                    local_cross_x100 = (uint32_t)i * 100U;
                } else {
                    uint32_t frac_x100 = (uint32_t)(((int32_t)avg_raw - prev) * 100 / (int32_t)den);
                    if (frac_x100 > 100U) {
                        frac_x100 = 100U;
                    }
                    local_cross_x100 = ((uint32_t)(i - 1U) * 100U) + frac_x100;
                }

                abs_cross_x100 = (g_ui.state.wave_abs_sample * 100U) + local_cross_x100;
                if (g_ui.state.wave_last_cross_x100 != 0U && abs_cross_x100 > g_ui.state.wave_last_cross_x100) {
                    uint64_t period_x100 = abs_cross_x100 - g_ui.state.wave_last_cross_x100;
                    uint32_t freq_x10 = (uint32_t)(((uint64_t)WAVE_SAMPLE_RATE_HZ * 1000ULL + (period_x100 / 2ULL)) / period_x100);

                    if (freq_x10 <= (WAVE_FREQ_MAX_HZ * 10U)) {
                        if (freq_x10 < (WAVE_LOW_FREQ_LIMIT_HZ * 10U)) {
                            uint32_t low_freq_x10 = estimate_low_frequency_x10(freq_x10);

                            if (low_freq_x10 != 0U) {
                                g_ui.state.wave_freq_calc_x10 = low_freq_x10;
                                g_ui.state.wave_freq_calc_valid = true;
                                push_frequency_sample(low_freq_x10);
                            }
                        } else if (g_ui.state.wave_freq_calc_valid) {
                            g_ui.state.wave_period_avg_x100 = 0U;
                            g_ui.state.wave_freq_calc_x10 = (g_ui.state.wave_freq_calc_x10 * 3U + freq_x10 + 2U) / 4U;
                            push_frequency_sample(g_ui.state.wave_freq_calc_x10);
                        } else {
                            g_ui.state.wave_period_avg_x100 = 0U;
                            g_ui.state.wave_freq_calc_x10 = freq_x10;
                            g_ui.state.wave_freq_calc_valid = true;
                            push_frequency_sample(g_ui.state.wave_freq_calc_x10);
                        }
                    }
                }

                g_ui.state.wave_last_cross_x100 = abs_cross_x100;
                g_ui.state.wave_above_high = true;
            }
        }
    } else {
        reset_frequency_display();
        g_ui.state.wave_above_high = false;
        g_ui.state.wave_last_cross_x100 = 0U;
    }

    g_ui.state.wave_abs_sample += count;
    update_waveform_obj();
}

uint8_t slave_ui_get_station_id(void)
{
    return g_ui.state.station_id;
}

bool slave_ui_group_enabled(void)
{
    return g_ui.state.group_enabled;
}
