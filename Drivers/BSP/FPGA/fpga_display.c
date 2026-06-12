#include "./BSP/FPGA/fpga_display.h"
#include "./BSP/LVGL/slave_ui.h"
#include "./SYSTEM/usart/usart.h"

#include <ctype.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

static uint32_t g_fpga_bad_lines = 0U;
static uint32_t g_fpga_sms_lines = 0U;
static uint32_t g_fpga_rx_lines = 0U;
static uint32_t g_fpga_log_lines = 0U;

static void fpga_send_command(const char *command)
{
    char line[24];
    int len;

    if (command == NULL) {
        return;
    }

    len = snprintf(line, sizeof(line), "%s\r\n", command);
    if (len <= 0) {
        return;
    }
    if (len >= (int)sizeof(line)) {
        len = (int)sizeof(line) - 1;
    }

    (void)HAL_UART_Transmit(&g_uart1_handle, (uint8_t *)line, (uint16_t)len, 100U);
}

static char *fpga_skip_space(char *s)
{
    while (*s == ' ' || *s == '\t')
    {
        s++;
    }

    return s;
}

static void fpga_trim_right(char *s)
{
    size_t len = strlen(s);

    while (len > 0U)
    {
        char ch = s[len - 1U];

        if (ch != '\r' && ch != '\n' && ch != ' ' && ch != '\t')
        {
            break;
        }

        s[len - 1U] = '\0';
        len--;
    }
}

static bool fpga_parse_u8(const char *s, uint8_t *value)
{
    unsigned long v = 0UL;
    uint8_t base = 10U;

    if (s == NULL || value == NULL || *s == '\0')
    {
        return false;
    }

    if ((s[0] == '0') && (s[1] == 'x' || s[1] == 'X'))
    {
        s += 2;
        base = 16U;
    }
    else if ((s[0] == 'F' || s[0] == 'f') && (s[1] == 'F' || s[1] == 'f') && s[2] == '\0')
    {
        *value = 0xFFU;
        return true;
    }

    while (*s != '\0')
    {
        uint8_t digit;
        char ch = *s++;

        if (ch >= '0' && ch <= '9')
        {
            digit = (uint8_t)(ch - '0');
        }
        else if (base == 16U && ch >= 'A' && ch <= 'F')
        {
            digit = (uint8_t)(ch - 'A' + 10);
        }
        else if (base == 16U && ch >= 'a' && ch <= 'f')
        {
            digit = (uint8_t)(ch - 'a' + 10);
        }
        else
        {
            return false;
        }

        if (digit >= base)
        {
            return false;
        }

        v = v * base + digit;
        if (v > 255UL)
        {
            return false;
        }
    }

    *value = (uint8_t)v;
    return true;
}

static bool fpga_parse_i16(const char *s, int16_t *value)
{
    int32_t sign = 1;
    int32_t v = 0;

    if (s == NULL || value == NULL || *s == '\0')
    {
        return false;
    }

    if (*s == '-')
    {
        sign = -1;
        s++;
    }

    if (*s == '\0')
    {
        return false;
    }

    while (*s != '\0')
    {
        if (*s < '0' || *s > '9')
        {
            return false;
        }

        v = v * 10 + (*s - '0');
        if (v > 32767)
        {
            return false;
        }
        s++;
    }

    v *= sign;
    if (v < -32768 || v > 32767)
    {
        return false;
    }

    *value = (int16_t)v;
    return true;
}

static bool fpga_text_char_valid(char ch)
{
    if (ch >= 'A' && ch <= 'Z')
    {
        return true;
    }

    if (ch >= '0' && ch <= '9')
    {
        return true;
    }

    switch (ch)
    {
        case ' ':
        case '.':
        case ',':
        case '?':
        case '!':
        case '-':
        case '/':
            return true;

        default:
            return false;
    }
}

static void fpga_filter_text(char *dst, size_t dst_size, const char *src)
{
    size_t n = 0U;

    if (dst == NULL || dst_size == 0U)
    {
        return;
    }

    if (src == NULL)
    {
        dst[0] = '\0';
        return;
    }

    while (*src != '\0' && n < dst_size - 1U)
    {
        char ch = *src++;

        if (ch >= 'a' && ch <= 'z')
        {
            ch = (char)(ch - 'a' + 'A');
        }

        if (fpga_text_char_valid(ch))
        {
            dst[n++] = ch;
        }
    }

    dst[n] = '\0';
}

static bool fpga_json_get_string(const char *line, const char *key, char *out, size_t out_size)
{
    char pattern[32];
    const char *p;
    size_t n = 0U;

    if (line == NULL || key == NULL || out == NULL || out_size == 0U) {
        return false;
    }

    snprintf(pattern, sizeof(pattern), "\"%s\":\"", key);
    p = strstr(line, pattern);
    if (p == NULL) {
        return false;
    }
    p += strlen(pattern);

    while (*p != '\0' && *p != '"' && n < out_size - 1U) {
        if (*p == '\\' && p[1] != '\0') {
            p++;
        }
        out[n++] = *p++;
    }
    out[n] = '\0';
    return true;
}

static bool fpga_json_get_int(const char *line, const char *key, int32_t *value)
{
    char pattern[32];
    const char *p;
    int32_t sign = 1;
    int32_t v = 0;
    bool any = false;

    if (line == NULL || key == NULL || value == NULL) {
        return false;
    }

    snprintf(pattern, sizeof(pattern), "\"%s\":", key);
    p = strstr(line, pattern);
    if (p == NULL) {
        return false;
    }
    p += strlen(pattern);

    if (*p == '-') {
        sign = -1;
        p++;
    }

    while (*p >= '0' && *p <= '9') {
        any = true;
        v = (v * 10) + (*p - '0');
        p++;
    }

    if (!any) {
        return false;
    }

    *value = v * sign;
    return true;
}

static bool fpga_json_get_bool(const char *line, const char *key, bool *value)
{
    char pattern[32];
    const char *p;

    if (line == NULL || key == NULL || value == NULL) {
        return false;
    }

    snprintf(pattern, sizeof(pattern), "\"%s\":", key);
    p = strstr(line, pattern);
    if (p == NULL) {
        return false;
    }
    p += strlen(pattern);

    if (strncmp(p, "true", 4U) == 0) {
        *value = true;
        return true;
    }
    if (strncmp(p, "false", 5U) == 0) {
        *value = false;
        return true;
    }
    return false;
}

static void fpga_json_log_summary(const char *line)
{
    char kind[24];
    char state[40];
    char result[40];
    int32_t attempt = 0;
    char msg[FPGA_DISPLAY_TEXT_MAX];

    if (!fpga_json_get_string(line, "kind", kind, sizeof(kind))) {
        return;
    }

    if (strcmp(kind, "attempt") == 0) {
        state[0] = '\0';
        (void)fpga_json_get_string(line, "state", state, sizeof(state));
        (void)fpga_json_get_int(line, "attempt", &attempt);
        snprintf(msg, sizeof(msg), "attempt %02ld: %s",
                 (long)attempt,
                 state[0] ? state : "...");
        slave_ui_set_capture_state(msg);
    } else if (strcmp(kind, "status") == 0) {
        result[0] = '\0';
        (void)fpga_json_get_string(line, "result", result, sizeof(result));
        snprintf(msg, sizeof(msg), "PYNQ %s", result[0] ? result : "status");
        slave_ui_set_pynq_status(msg);
    } else if (strcmp(kind, "test") == 0) {
        result[0] = '\0';
        state[0] = '\0';
        (void)fpga_json_get_string(line, "result", result, sizeof(result));
        (void)fpga_json_get_string(line, "state", state, sizeof(state));
        snprintf(msg, sizeof(msg), "TEST %s", result[0] ? result : (state[0] ? state : "--"));
        slave_ui_set_test_result(msg);
    }
}

static fpga_display_result_t fpga_handle_json(char *line)
{
    char kind[24];
    char payload[FPGA_DISPLAY_TEXT_MAX + 1U];
    char status[FPGA_DISPLAY_TEXT_MAX + 32U];
    int32_t addr = 0;
    int32_t length = 0;
    bool crc_ok = false;

    if (!fpga_json_get_string(line, "kind", kind, sizeof(kind))) {
        return FPGA_DISPLAY_RESULT_BAD_LINE;
    }

    if (strcmp(kind, "sms") == 0) {
        payload[0] = '\0';
        (void)fpga_json_get_string(line, "payload", payload, sizeof(payload));
        (void)fpga_json_get_int(line, "addr", &addr);
        (void)fpga_json_get_int(line, "length", &length);
        (void)fpga_json_get_bool(line, "crc_ok", &crc_ok);

        /* Try PYNQ group_call / addr_ok fields; fallback: addr==0xFF */
        bool json_group_call = false;
        bool addr_ok = false;
        (void)fpga_json_get_bool(line, "group_call", &json_group_call);
        (void)fpga_json_get_bool(line, "addr_ok", &addr_ok);
        bool group_call = json_group_call || ((uint8_t)addr == 0xFFU) || addr_ok;

        fpga_filter_text(payload, sizeof(payload), payload);
        if (payload[0] == '\0') {
            snprintf(payload, sizeof(payload), "(empty)");
        }

        /* Display if: group call, addr_ok, or addr matches local station */
        if (group_call || addr_ok || ((uint8_t)addr == slave_ui_get_station_id())) {
            slave_ui_set_sms(payload, (uint8_t)addr, group_call);
        } else {
            snprintf(status, sizeof(status), "DROP S%ld: %.80s",
                     (long)addr,
                     payload[0] ? payload : "(empty)");
            slave_ui_append_log(status);
        }

        g_fpga_sms_lines++;
        return FPGA_DISPLAY_RESULT_SMS;
    }

    /* Non-SMS JSON (status/test/attempt): raw echo already shows it; skip extra logging */
    if (strcmp(kind, "status") == 0) {
        fpga_json_log_summary(line);
        return FPGA_DISPLAY_RESULT_STATUS;
    }
    if (strcmp(kind, "test") == 0) {
        fpga_json_log_summary(line);
        return FPGA_DISPLAY_RESULT_TEST;
    }
    if (strcmp(kind, "attempt") == 0) {
        g_fpga_log_lines++;
        return FPGA_DISPLAY_RESULT_LOG;
    }

    return FPGA_DISPLAY_RESULT_LOG;
}

static fpga_display_result_t fpga_handle_sms(char *args)
{
    char *addr_s;
    char *text_s;
    char text[FPGA_DISPLAY_TEXT_MAX + 1U];
    char status[FPGA_DISPLAY_TEXT_MAX + 32U];
    uint8_t addr;
    bool group_call;

    addr_s = fpga_skip_space(args);
    text_s = strchr(addr_s, ',');
    if (text_s == NULL)
    {
        return FPGA_DISPLAY_RESULT_BAD_LINE;
    }

    *text_s++ = '\0';
    text_s = fpga_skip_space(text_s);
    fpga_trim_right(text_s);

    if (!fpga_parse_u8(addr_s, &addr))
    {
        return FPGA_DISPLAY_RESULT_BAD_LINE;
    }

    fpga_filter_text(text, sizeof(text), text_s);
    if (text[0] == '\0')
    {
        return FPGA_DISPLAY_RESULT_BAD_LINE;
    }

    group_call = (addr == 0xFFU);

    /* Group call always displays; normal call: addr must match local station */
    if (group_call || (addr == slave_ui_get_station_id())) {
        slave_ui_set_sms(text, addr, group_call);
    } else {
        snprintf(status, sizeof(status), "DROP S%u: %.80s", addr,
                 text[0] ? text : "(empty)");
        slave_ui_append_log(status);
    }

    g_fpga_sms_lines++;
    return FPGA_DISPLAY_RESULT_SMS;
}

static fpga_display_result_t fpga_handle_rx(char *args)
{
    char *tok[5];
    uint8_t bool_value[3];
    int16_t rssi;
    uint8_t af;

    tok[0] = fpga_skip_space(args);
    for (uint8_t i = 1; i < 5U; i++)
    {
        tok[i] = strchr(tok[i - 1U], ',');
        if (tok[i] == NULL)
        {
            return FPGA_DISPLAY_RESULT_BAD_LINE;
        }
        *tok[i]++ = '\0';
        tok[i] = fpga_skip_space(tok[i]);
    }

    fpga_trim_right(tok[4]);

    for (uint8_t i = 0; i < 3U; i++)
    {
        if (!fpga_parse_u8(tok[i], &bool_value[i]) || bool_value[i] > 1U)
        {
            return FPGA_DISPLAY_RESULT_BAD_LINE;
        }
    }

    if (!fpga_parse_i16(tok[3], &rssi) || !fpga_parse_u8(tok[4], &af) || af > 100U)
    {
        return FPGA_DISPLAY_RESULT_BAD_LINE;
    }

    slave_ui_set_rx_state(bool_value[0] != 0U,
                          bool_value[1] != 0U,
                          bool_value[2] != 0U,
                          rssi,
                          af);
    g_fpga_rx_lines++;
    return FPGA_DISPLAY_RESULT_RX;
}

static fpga_display_result_t fpga_handle_battery(char *args)
{
    char *pct_s;
    uint8_t pct;
    uint16_t mv = 0U;

    args = fpga_skip_space(args);
    pct_s = strchr(args, ',');
    if (pct_s == NULL)
    {
        return FPGA_DISPLAY_RESULT_BAD_LINE;
    }

    *pct_s++ = '\0';
    pct_s = fpga_skip_space(pct_s);
    fpga_trim_right(pct_s);

    for (char *p = args; *p != '\0'; p++)
    {
        if (*p < '0' || *p > '9')
        {
            return FPGA_DISPLAY_RESULT_BAD_LINE;
        }

        mv = (uint16_t)(mv * 10U + (uint16_t)(*p - '0'));
    }

    if (!fpga_parse_u8(pct_s, &pct) || pct > 100U)
    {
        return FPGA_DISPLAY_RESULT_BAD_LINE;
    }

    slave_ui_set_battery(mv, pct);
    return FPGA_DISPLAY_RESULT_BATTERY;
}

static fpga_display_result_t fpga_handle_log(char *args)
{
    args = fpga_skip_space(args);
    fpga_trim_right(args);
    if (args[0] == '\0') {
        return FPGA_DISPLAY_RESULT_BAD_LINE;
    }
    slave_ui_append_log(args);
    g_fpga_log_lines++;
    return FPGA_DISPLAY_RESULT_LOG;
}

static fpga_display_result_t fpga_handle_status(char *args)
{
    args = fpga_skip_space(args);
    fpga_trim_right(args);
    if (args[0] == '\0') {
        return FPGA_DISPLAY_RESULT_BAD_LINE;
    }
    slave_ui_set_pynq_status(args);
    slave_ui_append_log(args);
    return FPGA_DISPLAY_RESULT_STATUS;
}

static fpga_display_result_t fpga_handle_test(char *args)
{
    args = fpga_skip_space(args);
    fpga_trim_right(args);
    if (args[0] == '\0') {
        return FPGA_DISPLAY_RESULT_BAD_LINE;
    }
    slave_ui_set_test_result(args);
    slave_ui_append_log(args);
    return FPGA_DISPLAY_RESULT_TEST;
}

void FPGA_DisplayInit(void)
{
    g_fpga_bad_lines = 0U;
    g_fpga_sms_lines = 0U;
    g_fpga_rx_lines = 0U;
    g_fpga_log_lines = 0U;
    slave_ui_set_command_callback(fpga_send_command);
    slave_ui_set_pynq_status("BOOT WAIT");
    slave_ui_set_capture_state("IDLE");
    slave_ui_set_test_result("--");
}

fpga_display_result_t FPGA_DisplayProcessLine(const char *line)
{
    char buf[FPGA_DISPLAY_LINE_MAX];
    char *cmd;
    char *args;
    fpga_display_result_t result;

    if (line == NULL || line[0] == '\0')
    {
        return FPGA_DISPLAY_RESULT_NONE;
    }

    snprintf(buf, sizeof(buf), "%s", line);
    fpga_trim_right(buf);

    if (buf[0] == '{') {
        result = fpga_handle_json(buf);
        if (result == FPGA_DISPLAY_RESULT_BAD_LINE) {
            g_fpga_bad_lines++;
        }
        return result;
    }

    cmd = fpga_skip_space(buf);
    args = strchr(cmd, ',');
    if (args != NULL)
    {
        *args++ = '\0';
    }

    for (char *p = cmd; *p != '\0'; p++)
    {
        *p = (char)toupper((unsigned char)*p);
    }

    if (strcmp(cmd, "SMS") == 0 && args != NULL)
    {
        result = fpga_handle_sms(args);
    }
    else if (strcmp(cmd, "RX") == 0 && args != NULL)
    {
        result = fpga_handle_rx(args);
    }
    else if (strcmp(cmd, "BAT") == 0 && args != NULL)
    {
        result = fpga_handle_battery(args);
    }
    else if (strcmp(cmd, "STAT") == 0 && args != NULL)
    {
        char rx_line[48];

        snprintf(rx_line, sizeof(rx_line), "1,1,0,%s", args);
        result = fpga_handle_rx(rx_line);
    }
    else if (strcmp(cmd, "STATUS") == 0 && args != NULL)
    {
        result = fpga_handle_status(args);
    }
    else if (strcmp(cmd, "TEST") == 0 && args != NULL)
    {
        result = fpga_handle_test(args);
    }
    else if (strcmp(cmd, "LOG") == 0 && args != NULL)
    {
        result = fpga_handle_log(args);
    }
    else if (strcmp(cmd, "ERR") == 0 && args != NULL)
    {
        result = fpga_handle_log(args);
        slave_ui_set_capture_state("ERROR");
    }
    else
    {
        result = FPGA_DISPLAY_RESULT_BAD_LINE;
    }

    if (result == FPGA_DISPLAY_RESULT_BAD_LINE)
    {
        g_fpga_bad_lines++;
    }

    return result;
}

void FPGA_DisplayPollUsart1(void)
{
    char line[FPGA_DISPLAY_LINE_MAX];
    uint16_t len;

    if ((g_usart_rx_sta & 0x8000U) == 0U)
    {
        return;
    }

    __disable_irq();
    len = g_usart_rx_sta & 0x3FFFU;
    if (len >= FPGA_DISPLAY_LINE_MAX)
    {
        len = FPGA_DISPLAY_LINE_MAX - 1U;
    }
    memcpy(line, g_usart_rx_buf, len);
    line[len] = '\0';
    g_usart_rx_sta = 0U;
    __enable_irq();

    (void)FPGA_DisplayProcessLine(line);
}
