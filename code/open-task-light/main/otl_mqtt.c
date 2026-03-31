#include "otl_mqtt.h"

#include <ctype.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

#include "sdkconfig.h"

#if CONFIG_OTL_HA_MQTT_ENABLE

#include "otl_build_info.h"
#include "esp_app_desc.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "mqtt_client.h"
#include "otl_net.h"
#include "otl_runtime.h"

static const char *TAG = "otl_mqtt";

#define OTL_MQTT_TOPIC_MAX           256
#define OTL_MQTT_PAYLOAD_MAX         2048
#define OTL_MQTT_DEVICE_ID_MAX       48
#define OTL_MQTT_MAX_BRIGHTNESS_PCT  95.0f
#define OTL_MQTT_WARM_KELVIN         2700.0f
#define OTL_MQTT_COOL_KELVIN         5000.0f
#define OTL_MQTT_WARM_MIRED          (1000000.0f / OTL_MQTT_WARM_KELVIN)
#define OTL_MQTT_COOL_MIRED          (1000000.0f / OTL_MQTT_COOL_KELVIN)
#define OTL_MQTT_CM_PER_FOOT         30.48f
#define OTL_MQTT_TASK_NAME           "otl_mqtt"
#define OTL_MQTT_TASK_STACK_SIZE     6144
#define OTL_MQTT_TASK_PRIORITY       4

typedef struct {
    esp_mqtt_client_handle_t client;
    bool started;
    bool start_requested;
    bool connected;
    bool state_listener_registered;
    bool telemetry_listener_registered;
    bool settings_listener_registered;
    bool event_listener_registered;
    char device_id[OTL_MQTT_DEVICE_ID_MAX];
    char topic_root[OTL_MQTT_TOPIC_MAX];
    char availability_topic[OTL_MQTT_TOPIC_MAX];
    char event_stream_topic[OTL_MQTT_TOPIC_MAX];
    char light_state_topic[OTL_MQTT_TOPIC_MAX];
    char light_command_topic[OTL_MQTT_TOPIC_MAX];
    char brightness_state_topic[OTL_MQTT_TOPIC_MAX];
    char brightness_command_topic[OTL_MQTT_TOPIC_MAX];
    char color_temp_state_topic[OTL_MQTT_TOPIC_MAX];
    char color_temp_command_topic[OTL_MQTT_TOPIC_MAX];
    char light_discovery_topic[OTL_MQTT_TOPIC_MAX];
#if CONFIG_OTL_PRESENCE_SENSOR
    char occupancy_state_topic[OTL_MQTT_TOPIC_MAX];
    char occupancy_discovery_topic[OTL_MQTT_TOPIC_MAX];
    char motion_state_topic[OTL_MQTT_TOPIC_MAX];
    char motion_discovery_topic[OTL_MQTT_TOPIC_MAX];
    char motion_distance_state_topic[OTL_MQTT_TOPIC_MAX];
    char motion_distance_discovery_topic[OTL_MQTT_TOPIC_MAX];
    char stationary_distance_state_topic[OTL_MQTT_TOPIC_MAX];
    char stationary_distance_discovery_topic[OTL_MQTT_TOPIC_MAX];
#endif
    char ambient_lux_state_topic[OTL_MQTT_TOPIC_MAX];
    char ambient_lux_discovery_topic[OTL_MQTT_TOPIC_MAX];
    char ntc_temp_state_topic[OTL_MQTT_TOPIC_MAX];
    char ntc_temp_discovery_topic[OTL_MQTT_TOPIC_MAX];
    char chip_temp_state_topic[OTL_MQTT_TOPIC_MAX];
    char chip_temp_discovery_topic[OTL_MQTT_TOPIC_MAX];
    char thermal_cap_state_topic[OTL_MQTT_TOPIC_MAX];
    char thermal_cap_discovery_topic[OTL_MQTT_TOPIC_MAX];
    char thermal_limited_state_topic[OTL_MQTT_TOPIC_MAX];
    char thermal_limited_discovery_topic[OTL_MQTT_TOPIC_MAX];
    char wifi_rssi_state_topic[OTL_MQTT_TOPIC_MAX];
    char wifi_rssi_discovery_topic[OTL_MQTT_TOPIC_MAX];
    char led_thermal_limit_state_topic[OTL_MQTT_TOPIC_MAX];
    char led_thermal_limit_command_topic[OTL_MQTT_TOPIC_MAX];
    char led_thermal_limit_discovery_topic[OTL_MQTT_TOPIC_MAX];
#if CONFIG_OTL_CIRCADIAN_ENABLE
    char circadian_enabled_state_topic[OTL_MQTT_TOPIC_MAX];
    char circadian_enabled_command_topic[OTL_MQTT_TOPIC_MAX];
    char circadian_enabled_discovery_topic[OTL_MQTT_TOPIC_MAX];
    char circadian_coolest_state_topic[OTL_MQTT_TOPIC_MAX];
    char circadian_coolest_command_topic[OTL_MQTT_TOPIC_MAX];
    char circadian_coolest_discovery_topic[OTL_MQTT_TOPIC_MAX];
    char circadian_warmest_state_topic[OTL_MQTT_TOPIC_MAX];
    char circadian_warmest_command_topic[OTL_MQTT_TOPIC_MAX];
    char circadian_warmest_discovery_topic[OTL_MQTT_TOPIC_MAX];
    char circadian_morning_ramp_state_topic[OTL_MQTT_TOPIC_MAX];
    char circadian_morning_ramp_command_topic[OTL_MQTT_TOPIC_MAX];
    char circadian_morning_ramp_discovery_topic[OTL_MQTT_TOPIC_MAX];
#endif
    char verbose_diag_state_topic[OTL_MQTT_TOPIC_MAX];
    char verbose_diag_command_topic[OTL_MQTT_TOPIC_MAX];
    char verbose_diag_discovery_topic[OTL_MQTT_TOPIC_MAX];
    char status_log_state_topic[OTL_MQTT_TOPIC_MAX];
    char status_log_command_topic[OTL_MQTT_TOPIC_MAX];
    char status_log_discovery_topic[OTL_MQTT_TOPIC_MAX];
    char sensor_debug_state_topic[OTL_MQTT_TOPIC_MAX];
    char sensor_debug_command_topic[OTL_MQTT_TOPIC_MAX];
    char sensor_debug_discovery_topic[OTL_MQTT_TOPIC_MAX];
    char touch_event_log_state_topic[OTL_MQTT_TOPIC_MAX];
    char touch_event_log_command_topic[OTL_MQTT_TOPIC_MAX];
    char touch_event_log_discovery_topic[OTL_MQTT_TOPIC_MAX];
    char touch_cal_log_state_topic[OTL_MQTT_TOPIC_MAX];
    char touch_cal_log_command_topic[OTL_MQTT_TOPIC_MAX];
    char touch_cal_log_discovery_topic[OTL_MQTT_TOPIC_MAX];
    char touch_raw_log_state_topic[OTL_MQTT_TOPIC_MAX];
    char touch_raw_log_command_topic[OTL_MQTT_TOPIC_MAX];
    char touch_raw_log_discovery_topic[OTL_MQTT_TOPIC_MAX];
    char pwm_log_state_topic[OTL_MQTT_TOPIC_MAX];
    char pwm_log_command_topic[OTL_MQTT_TOPIC_MAX];
    char pwm_log_discovery_topic[OTL_MQTT_TOPIC_MAX];
#if CONFIG_OTL_PRESENCE_SENSOR
    char radar_log_state_topic[OTL_MQTT_TOPIC_MAX];
    char radar_log_command_topic[OTL_MQTT_TOPIC_MAX];
    char radar_log_discovery_topic[OTL_MQTT_TOPIC_MAX];
    char radar_motion_max_distance_state_topic[OTL_MQTT_TOPIC_MAX];
    char radar_motion_max_distance_command_topic[OTL_MQTT_TOPIC_MAX];
    char radar_motion_max_distance_discovery_topic[OTL_MQTT_TOPIC_MAX];
    char radar_stationary_max_distance_state_topic[OTL_MQTT_TOPIC_MAX];
    char radar_stationary_max_distance_command_topic[OTL_MQTT_TOPIC_MAX];
    char radar_stationary_max_distance_discovery_topic[OTL_MQTT_TOPIC_MAX];
    char occupancy_auto_off_state_topic[OTL_MQTT_TOPIC_MAX];
    char occupancy_auto_off_command_topic[OTL_MQTT_TOPIC_MAX];
    char occupancy_auto_off_discovery_topic[OTL_MQTT_TOPIC_MAX];
#endif
    char firmware_version_state_topic[OTL_MQTT_TOPIC_MAX];
    char firmware_version_discovery_topic[OTL_MQTT_TOPIC_MAX];
    char firmware_built_state_topic[OTL_MQTT_TOPIC_MAX];
    char firmware_built_discovery_topic[OTL_MQTT_TOPIC_MAX];
    char firmware_idf_state_topic[OTL_MQTT_TOPIC_MAX];
    char firmware_idf_discovery_topic[OTL_MQTT_TOPIC_MAX];
    char firmware_target_state_topic[OTL_MQTT_TOPIC_MAX];
    char firmware_target_discovery_topic[OTL_MQTT_TOPIC_MAX];
    char firmware_homekit_state_topic[OTL_MQTT_TOPIC_MAX];
    char firmware_homekit_discovery_topic[OTL_MQTT_TOPIC_MAX];
    char firmware_presence_state_topic[OTL_MQTT_TOPIC_MAX];
    char firmware_presence_discovery_topic[OTL_MQTT_TOPIC_MAX];
    char firmware_circadian_state_topic[OTL_MQTT_TOPIC_MAX];
    char firmware_circadian_discovery_topic[OTL_MQTT_TOPIC_MAX];
    char last_event_state_topic[OTL_MQTT_TOPIC_MAX];
    char last_event_discovery_topic[OTL_MQTT_TOPIC_MAX];
} otl_mqtt_ctx_t;

static otl_mqtt_ctx_t s_mqtt = {0};

typedef esp_err_t (*otl_mqtt_bool_setting_setter_fn)(bool enabled, otl_change_source_t source);

static esp_err_t otl_mqtt_build_topics(void);
static esp_err_t otl_mqtt_publish_discovery_payload(const char *topic, const char *payload);
static void otl_mqtt_publish(const char *topic, const char *payload, bool retain);
static void otl_mqtt_cleanup_stale_discovery(void);
static void otl_mqtt_event_handler(void *handler_args,
                                   esp_event_base_t base,
                                   int32_t event_id,
                                   void *event_data);

static float otl_mqtt_clampf(float value, float lo, float hi)
{
    if (value < lo) {
        return lo;
    }
    if (value > hi) {
        return hi;
    }
    return value;
}

static bool otl_mqtt_topic_matches(esp_mqtt_event_handle_t event, const char *topic)
{
    size_t topic_len = strlen(topic);
    return (event->topic_len == (int)topic_len) &&
           (memcmp(event->topic, topic, topic_len) == 0);
}

static bool otl_mqtt_copy_payload(esp_mqtt_event_handle_t event, char *buf, size_t buf_len)
{
    if (buf == NULL || buf_len == 0 || event->data_len <= 0 || (size_t)event->data_len >= buf_len) {
        return false;
    }

    memcpy(buf, event->data, event->data_len);
    buf[event->data_len] = '\0';
    return true;
}

static void otl_mqtt_trim(char *buf)
{
    size_t len = 0;
    size_t start = 0;

    if (buf == NULL) {
        return;
    }

    len = strlen(buf);
    while (start < len && isspace((unsigned char)buf[start])) {
        ++start;
    }
    while (len > start && isspace((unsigned char)buf[len - 1])) {
        --len;
    }
    if (start > 0) {
        memmove(buf, buf + start, len - start);
    }
    buf[len - start] = '\0';
}

static bool otl_mqtt_parse_int_payload(esp_mqtt_event_handle_t event, int *value_out)
{
    char payload[32] = {0};
    char *endptr = NULL;
    long parsed = 0;

    if (value_out == NULL || !otl_mqtt_copy_payload(event, payload, sizeof(payload))) {
        return false;
    }

    otl_mqtt_trim(payload);
    parsed = strtol(payload, &endptr, 10);
    if (endptr == payload || *endptr != '\0') {
        return false;
    }

    *value_out = (int)parsed;
    return true;
}

static bool otl_mqtt_parse_float_payload(esp_mqtt_event_handle_t event, float *value_out)
{
    char payload[32] = {0};
    char *endptr = NULL;
    float parsed = 0.0f;

    if (value_out == NULL || !otl_mqtt_copy_payload(event, payload, sizeof(payload))) {
        return false;
    }

    otl_mqtt_trim(payload);
    parsed = strtof(payload, &endptr);
    if (endptr == payload || *endptr != '\0' || !isfinite(parsed)) {
        return false;
    }

    *value_out = parsed;
    return true;
}

static bool otl_mqtt_parse_on_off_payload(esp_mqtt_event_handle_t event, bool *value_out)
{
    char payload[16] = {0};

    if (value_out == NULL || !otl_mqtt_copy_payload(event, payload, sizeof(payload))) {
        return false;
    }

    otl_mqtt_trim(payload);
    if (strcasecmp(payload, "ON") == 0) {
        *value_out = true;
        return true;
    }
    if (strcasecmp(payload, "OFF") == 0) {
        *value_out = false;
        return true;
    }

    return false;
}

static void otl_mqtt_publish_bool_state(const char *topic, bool enabled)
{
    otl_mqtt_publish(topic, enabled ? "ON" : "OFF", true);
}

static esp_err_t otl_mqtt_format_topic(char *dst, size_t dst_size, const char *fmt, ...)
{
    va_list args;
    va_list args_copy;
    int needed = 0;

    if (dst == NULL || dst_size == 0 || fmt == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    va_start(args, fmt);
    va_copy(args_copy, args);
    needed = vsnprintf(NULL, 0, fmt, args_copy);
    va_end(args_copy);
    if (needed < 0 || (size_t)needed >= dst_size) {
        va_end(args);
        return ESP_ERR_INVALID_SIZE;
    }

    (void)vsnprintf(dst, dst_size, fmt, args);
    va_end(args);
    return ESP_OK;
}

static char *otl_mqtt_alloc_printf(const char *fmt, ...)
{
    va_list args;
    va_list args_copy;
    int needed = 0;
    char *buf = NULL;

    if (fmt == NULL) {
        return NULL;
    }

    va_start(args, fmt);
    va_copy(args_copy, args);
    needed = vsnprintf(NULL, 0, fmt, args_copy);
    va_end(args_copy);
    if (needed < 0) {
        va_end(args);
        return NULL;
    }

    buf = malloc((size_t)needed + 1U);
    if (buf == NULL) {
        va_end(args);
        return NULL;
    }

    (void)vsnprintf(buf, (size_t)needed + 1U, fmt, args);
    va_end(args);
    return buf;
}

static char *otl_mqtt_json_escape(const char *src)
{
    size_t src_len = 0;
    size_t extra = 0;
    char *dst = NULL;
    char *out = NULL;

    if (src == NULL) {
        return NULL;
    }

    src_len = strlen(src);
    for (size_t i = 0; i < src_len; ++i) {
        unsigned char c = (unsigned char)src[i];
        if (c == '"' || c == '\\') {
            extra += 1;
        }
    }

    dst = calloc(1, src_len + extra + 1U);
    if (dst == NULL) {
        return NULL;
    }

    out = dst;
    for (size_t i = 0; i < src_len; ++i) {
        unsigned char c = (unsigned char)src[i];
        if (c == '"' || c == '\\') {
            *out++ = '\\';
        }
        *out++ = (char)c;
    }
    *out = '\0';
    return dst;
}

static void otl_mqtt_publish_switch(const char *name,
                                    const char *unique_suffix,
                                    const char *command_topic,
                                    const char *state_topic,
                                    const char *discovery_topic,
                                    const char *icon,
                                    const char *entity_category,
                                    const char *device_json)
{
    char *payload = NULL;
    const char *icon_json = "";

    if (name == NULL || unique_suffix == NULL || command_topic == NULL || state_topic == NULL ||
        discovery_topic == NULL || entity_category == NULL || device_json == NULL) {
        return;
    }

    if (icon != NULL && icon[0] != '\0') {
        icon_json = otl_mqtt_alloc_printf("\"icon\":\"%s\",", icon);
        if (icon_json == NULL) {
            return;
        }
    }

    payload = otl_mqtt_alloc_printf(
        "{"
        "\"name\":\"%s\","
        "\"unique_id\":\"%s_%s\","
        "\"command_topic\":\"%s\","
        "\"state_topic\":\"%s\","
        "\"payload_on\":\"ON\","
        "\"payload_off\":\"OFF\","
        "\"state_on\":\"ON\","
        "\"state_off\":\"OFF\","
        "%s"
        "\"entity_category\":\"%s\","
        "\"availability_topic\":\"%s\","
        "\"payload_available\":\"online\","
        "\"payload_not_available\":\"offline\","
        "%s"
        "}",
        name,
        s_mqtt.device_id,
        unique_suffix,
        command_topic,
        state_topic,
        icon_json,
        entity_category,
        s_mqtt.availability_topic,
        device_json);
    if (icon_json != NULL && icon_json[0] != '\0') {
        free((void *)icon_json);
    }
    if (payload != NULL) {
        (void)otl_mqtt_publish_discovery_payload(discovery_topic, payload);
        free(payload);
    }
}

static void otl_mqtt_publish_text_sensor(const char *name,
                                         const char *unique_suffix,
                                         const char *state_topic,
                                         const char *discovery_topic,
                                         const char *icon,
                                         const char *entity_category,
                                         const char *device_json)
{
    char *payload = NULL;
    const char *icon_json = "";

    if (name == NULL || unique_suffix == NULL || state_topic == NULL || discovery_topic == NULL ||
        entity_category == NULL || device_json == NULL) {
        return;
    }

    if (icon != NULL && icon[0] != '\0') {
        icon_json = otl_mqtt_alloc_printf("\"icon\":\"%s\",", icon);
        if (icon_json == NULL) {
            return;
        }
    }

    payload = otl_mqtt_alloc_printf(
        "{"
        "\"name\":\"%s\","
        "\"unique_id\":\"%s_%s\","
        "\"state_topic\":\"%s\","
        "%s"
        "\"entity_category\":\"%s\","
        "\"availability_topic\":\"%s\","
        "\"payload_available\":\"online\","
        "\"payload_not_available\":\"offline\","
        "%s"
        "}",
        name,
        s_mqtt.device_id,
        unique_suffix,
        state_topic,
        icon_json,
        entity_category,
        s_mqtt.availability_topic,
        device_json);
    if (icon_json != NULL && icon_json[0] != '\0') {
        free((void *)icon_json);
    }
    if (payload != NULL) {
        (void)otl_mqtt_publish_discovery_payload(discovery_topic, payload);
        free(payload);
    }
}

static void otl_mqtt_publish_binary_sensor(const char *name,
                                           const char *unique_suffix,
                                           const char *state_topic,
                                           const char *discovery_topic,
                                           const char *icon,
                                           const char *entity_category,
                                           const char *device_json)
{
    char *payload = NULL;
    const char *icon_json = "";

    if (name == NULL || unique_suffix == NULL || state_topic == NULL || discovery_topic == NULL ||
        entity_category == NULL || device_json == NULL) {
        return;
    }

    if (icon != NULL && icon[0] != '\0') {
        icon_json = otl_mqtt_alloc_printf("\"icon\":\"%s\",", icon);
        if (icon_json == NULL) {
            return;
        }
    }

    payload = otl_mqtt_alloc_printf(
        "{"
        "\"name\":\"%s\","
        "\"unique_id\":\"%s_%s\","
        "\"state_topic\":\"%s\","
        "\"payload_on\":\"ON\","
        "\"payload_off\":\"OFF\","
        "%s"
        "\"entity_category\":\"%s\","
        "\"availability_topic\":\"%s\","
        "\"payload_available\":\"online\","
        "\"payload_not_available\":\"offline\","
        "%s"
        "}",
        name,
        s_mqtt.device_id,
        unique_suffix,
        state_topic,
        icon_json,
        entity_category,
        s_mqtt.availability_topic,
        device_json);
    if (icon_json != NULL && icon_json[0] != '\0') {
        free((void *)icon_json);
    }
    if (payload != NULL) {
        (void)otl_mqtt_publish_discovery_payload(discovery_topic, payload);
        free(payload);
    }
}

static int otl_mqtt_brightness_to_ha(float brightness_percent)
{
    float ratio = otl_mqtt_clampf(brightness_percent, 0.0f, OTL_MQTT_MAX_BRIGHTNESS_PCT);
    ratio /= OTL_MQTT_MAX_BRIGHTNESS_PCT;
    return (int)lroundf(ratio * 255.0f);
}

static float otl_mqtt_brightness_from_ha(int brightness_ha)
{
    int clamped = brightness_ha;

    if (clamped < 1) {
        clamped = 1;
    }
    if (clamped > 255) {
        clamped = 255;
    }

    return ((float)clamped / 255.0f) * OTL_MQTT_MAX_BRIGHTNESS_PCT;
}

static int otl_mqtt_temp_ratio_to_mired(float temp_ratio)
{
    float ratio = otl_mqtt_clampf(temp_ratio, 0.0f, 1.0f);
    float mired = OTL_MQTT_WARM_MIRED - (ratio * (OTL_MQTT_WARM_MIRED - OTL_MQTT_COOL_MIRED));
    return (int)lroundf(mired);
}

static float otl_mqtt_mired_to_temp_ratio(int mired)
{
    float clamped = (float)mired;

    if (clamped < OTL_MQTT_COOL_MIRED) {
        clamped = OTL_MQTT_COOL_MIRED;
    }
    if (clamped > OTL_MQTT_WARM_MIRED) {
        clamped = OTL_MQTT_WARM_MIRED;
    }

    return (OTL_MQTT_WARM_MIRED - clamped) / (OTL_MQTT_WARM_MIRED - OTL_MQTT_COOL_MIRED);
}

#if CONFIG_OTL_PRESENCE_SENSOR
static float otl_mqtt_cm_to_feet(float cm)
{
    return cm / OTL_MQTT_CM_PER_FOOT;
}

static int otl_mqtt_feet_to_cm(float feet)
{
    return (int)lroundf(feet * OTL_MQTT_CM_PER_FOOT);
}
#endif

static void otl_mqtt_publish(const char *topic, const char *payload, bool retain)
{
    if (!s_mqtt.connected || s_mqtt.client == NULL || topic == NULL || payload == NULL) {
        return;
    }

    if (esp_mqtt_client_publish(s_mqtt.client, topic, payload, 0, 1, retain) < 0) {
        ESP_LOGW(TAG, "Publish failed for topic %s", topic);
    }
}

static void otl_mqtt_publish_int(const char *topic, int value, bool retain)
{
    char payload[16] = {0};

    snprintf(payload, sizeof(payload), "%d", value);
    otl_mqtt_publish(topic, payload, retain);
}

static void otl_mqtt_publish_float(const char *topic, float value, int decimals, bool retain)
{
    char payload[32] = {0};
    char fmt[8] = {0};

    if (topic == NULL || isnan(value)) {
        return;
    }

    snprintf(fmt, sizeof(fmt), "%%.%df", decimals);
    snprintf(payload, sizeof(payload), fmt, value);
    otl_mqtt_publish(topic, payload, retain);
}

static char *otl_mqtt_alloc_device_json(void)
{
    const esp_app_desc_t *app_desc = esp_app_get_description();
    return otl_mqtt_alloc_printf(
        "\"device\":{"
        "\"identifiers\":[\"%s\"],"
        "\"name\":\"%s\","
        "\"manufacturer\":\"Basement Labs\","
        "\"model\":\"Open Task Light\","
        "\"sw_version\":\"%s\""
        "}",
        s_mqtt.device_id,
        CONFIG_OTL_MQTT_DEVICE_NAME,
        app_desc->version);
}

static void otl_mqtt_publish_state(const otl_public_state_t *state)
{
    if (state == NULL) {
        return;
    }

    otl_mqtt_publish(s_mqtt.light_state_topic, state->is_on ? "ON" : "OFF", true);
    otl_mqtt_publish_int(s_mqtt.brightness_state_topic,
                         otl_mqtt_brightness_to_ha(state->brightness_percent),
                         true);
    otl_mqtt_publish_int(s_mqtt.color_temp_state_topic,
                         otl_mqtt_temp_ratio_to_mired(state->temp_ratio),
                         true);
#if CONFIG_OTL_PRESENCE_SENSOR
    otl_mqtt_publish(s_mqtt.occupancy_state_topic, state->presence ? "ON" : "OFF", true);
#endif
}

static void otl_mqtt_publish_telemetry(const otl_telemetry_t *telemetry)
{
    if (telemetry == NULL) {
        return;
    }

    otl_mqtt_publish_float(s_mqtt.ambient_lux_state_topic, telemetry->ambient_lux, 0, true);
    otl_mqtt_publish_float(s_mqtt.ntc_temp_state_topic, telemetry->ntc_temp_c, 1, true);
    otl_mqtt_publish_float(s_mqtt.chip_temp_state_topic, telemetry->chip_temp_c, 1, true);
    otl_mqtt_publish_float(s_mqtt.thermal_cap_state_topic,
                           telemetry->thermal_brightness_cap_percent,
                           0,
                           true);
    otl_mqtt_publish(s_mqtt.thermal_limited_state_topic,
                     telemetry->thermal_limited ? "ON" : "OFF",
                     true);
#if CONFIG_OTL_PRESENCE_SENSOR
    otl_mqtt_publish(s_mqtt.motion_state_topic,
                     telemetry->radar_motion_detected ? "ON" : "OFF",
                     true);
    otl_mqtt_publish_float(s_mqtt.motion_distance_state_topic,
                           otl_mqtt_cm_to_feet((float)telemetry->radar_motion_distance_cm),
                           1,
                           true);
    otl_mqtt_publish_float(s_mqtt.stationary_distance_state_topic,
                           otl_mqtt_cm_to_feet((float)telemetry->radar_stationary_distance_cm),
                           1,
                           true);
#endif
    if (telemetry->wifi_connected) {
        otl_mqtt_publish_int(s_mqtt.wifi_rssi_state_topic, telemetry->wifi_rssi_dbm, true);
    }
}

static void otl_mqtt_publish_settings(const otl_runtime_settings_t *settings)
{
    if (settings == NULL) {
        return;
    }

#if CONFIG_OTL_CIRCADIAN_ENABLE
    otl_mqtt_publish(s_mqtt.circadian_enabled_state_topic,
                     settings->circadian_enabled ? "ON" : "OFF",
                     true);
    otl_mqtt_publish(s_mqtt.circadian_coolest_state_topic,
                     settings->circadian_coolest_time,
                     true);
    otl_mqtt_publish(s_mqtt.circadian_warmest_state_topic,
                     settings->circadian_warmest_time,
                     true);
    otl_mqtt_publish_int(s_mqtt.circadian_morning_ramp_state_topic,
                         settings->circadian_morning_ramp_minutes,
                         true);
#endif
    otl_mqtt_publish_float(s_mqtt.led_thermal_limit_state_topic,
                           settings->led_thermal_limit_c,
                           0,
                           true);
    otl_mqtt_publish_bool_state(s_mqtt.verbose_diag_state_topic,
                                settings->verbose_diagnostics_enabled);
    otl_mqtt_publish_bool_state(s_mqtt.status_log_state_topic,
                                settings->status_logging_enabled);
    otl_mqtt_publish_bool_state(s_mqtt.sensor_debug_state_topic,
                                settings->sensor_debug_logging_enabled);
    otl_mqtt_publish_bool_state(s_mqtt.touch_event_log_state_topic,
                                settings->touch_event_logging_enabled);
    otl_mqtt_publish_bool_state(s_mqtt.touch_cal_log_state_topic,
                                settings->touch_calibration_logging_enabled);
    otl_mqtt_publish_bool_state(s_mqtt.touch_raw_log_state_topic,
                                settings->touch_raw_logging_enabled);
    otl_mqtt_publish_bool_state(s_mqtt.pwm_log_state_topic,
                                settings->pwm_duty_logging_enabled);
#if CONFIG_OTL_PRESENCE_SENSOR
    otl_mqtt_publish_bool_state(s_mqtt.radar_log_state_topic,
                                settings->radar_status_logging_enabled);
    otl_mqtt_publish_float(s_mqtt.radar_motion_max_distance_state_topic,
                           otl_mqtt_cm_to_feet((float)settings->radar_motion_max_distance_cm),
                           1,
                           true);
    otl_mqtt_publish_float(s_mqtt.radar_stationary_max_distance_state_topic,
                           otl_mqtt_cm_to_feet((float)settings->radar_stationary_max_distance_cm),
                           1,
                           true);
    otl_mqtt_publish_bool_state(s_mqtt.occupancy_auto_off_state_topic,
                                settings->occupancy_auto_off_enabled);
#endif
}

static void otl_mqtt_publish_event(const otl_runtime_event_t *event)
{
    char *ts_escaped = NULL;
    char *category_escaped = NULL;
    char *message_escaped = NULL;
    char *json_payload = NULL;
    char *last_event_payload = NULL;

    if (event == NULL) {
        return;
    }

    ts_escaped = otl_mqtt_json_escape(event->timestamp);
    category_escaped = otl_mqtt_json_escape(event->category);
    message_escaped = otl_mqtt_json_escape(event->message);
    if (ts_escaped == NULL || category_escaped == NULL || message_escaped == NULL) {
        goto cleanup;
    }

    json_payload = otl_mqtt_alloc_printf(
        "{"
        "\"timestamp\":\"%s\","
        "\"level\":\"%s\","
        "\"category\":\"%s\","
        "\"message\":\"%s\""
        "}",
        ts_escaped,
        otl_event_level_to_string(event->level),
        category_escaped,
        message_escaped);
    last_event_payload = otl_mqtt_alloc_printf("%s %s/%s %s",
                                               event->timestamp,
                                               otl_event_level_to_string(event->level),
                                               event->category,
                                               event->message);
    if (json_payload == NULL || last_event_payload == NULL) {
        goto cleanup;
    }

    otl_mqtt_publish(s_mqtt.event_stream_topic, json_payload, false);
    otl_mqtt_publish(s_mqtt.last_event_state_topic, last_event_payload, true);

cleanup:
    free(ts_escaped);
    free(category_escaped);
    free(message_escaped);
    free(json_payload);
    free(last_event_payload);
}

static void otl_mqtt_publish_build_info(void)
{
    const esp_app_desc_t *app_desc = esp_app_get_description();
    otl_mqtt_publish(s_mqtt.firmware_built_state_topic, OTL_BUILD_TIMESTAMP, true);

    if (app_desc != NULL) {
        otl_mqtt_publish(s_mqtt.firmware_version_state_topic, app_desc->version, true);
        otl_mqtt_publish(s_mqtt.firmware_idf_state_topic, app_desc->idf_ver, true);
    }
    otl_mqtt_publish(s_mqtt.firmware_target_state_topic, CONFIG_IDF_TARGET, true);
    otl_mqtt_publish_bool_state(s_mqtt.firmware_homekit_state_topic,
#if CONFIG_OTL_HOMEKIT_ENABLE
                                true
#else
                                false
#endif
    );
    otl_mqtt_publish_bool_state(s_mqtt.firmware_presence_state_topic,
#if CONFIG_OTL_PRESENCE_SENSOR
                                true
#else
                                false
#endif
    );
    otl_mqtt_publish_bool_state(s_mqtt.firmware_circadian_state_topic,
#if CONFIG_OTL_CIRCADIAN_ENABLE
                                true
#else
                                false
#endif
    );
}

static esp_err_t otl_mqtt_publish_discovery_payload(const char *topic, const char *payload)
{
    if (topic == NULL || payload == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    otl_mqtt_publish(topic, payload, true);
    return ESP_OK;
}

static void otl_mqtt_cleanup_stale_discovery(void)
{
#if !CONFIG_OTL_PRESENCE_SENSOR
    static const char *const stale_presence_topics[] = {
        "%s/binary_sensor/%s_occupancy/config",
        "%s/binary_sensor/%s_motion/config",
        "%s/sensor/%s_motion_distance/config",
        "%s/sensor/%s_stationary_distance/config",
        "%s/number/%s_motion_max_distance/config",
        "%s/number/%s_stationary_max_distance/config",
        "%s/switch/%s_radar_status_logs/config",
        "%s/switch/%s_occupancy_auto_off/config",
    };
    char topic[OTL_MQTT_TOPIC_MAX] = {0};

    for (size_t i = 0; i < sizeof(stale_presence_topics) / sizeof(stale_presence_topics[0]); ++i) {
        if (otl_mqtt_format_topic(topic,
                                  sizeof(topic),
                                  stale_presence_topics[i],
                                  CONFIG_OTL_MQTT_DISCOVERY_PREFIX,
                                  s_mqtt.device_id) == ESP_OK) {
            otl_mqtt_publish(topic, "", true);
        }
    }
#endif
}

static void otl_mqtt_publish_discovery(void)
{
    char *device_json = otl_mqtt_alloc_device_json();
    char *payload = NULL;

    if (device_json == NULL) {
        ESP_LOGE(TAG, "Failed to allocate MQTT device metadata");
        return;
    }

    payload = otl_mqtt_alloc_printf(
        "{"
        "\"name\":\"Task Light\","
        "\"unique_id\":\"%s_light\","
        "\"command_topic\":\"%s\","
        "\"state_topic\":\"%s\","
        "\"brightness_command_topic\":\"%s\","
        "\"brightness_state_topic\":\"%s\","
        "\"color_temp_command_topic\":\"%s\","
        "\"color_temp_state_topic\":\"%s\","
        "\"min_mireds\":%d,"
        "\"max_mireds\":%d,"
        "\"availability_topic\":\"%s\","
        "\"payload_available\":\"online\","
        "\"payload_not_available\":\"offline\","
        "%s"
        "}",
        s_mqtt.device_id,
        s_mqtt.light_command_topic,
        s_mqtt.light_state_topic,
        s_mqtt.brightness_command_topic,
        s_mqtt.brightness_state_topic,
        s_mqtt.color_temp_command_topic,
        s_mqtt.color_temp_state_topic,
        (int)lroundf(OTL_MQTT_COOL_MIRED),
        (int)lroundf(OTL_MQTT_WARM_MIRED),
        s_mqtt.availability_topic,
        device_json);
    if (payload != NULL) {
        (void)otl_mqtt_publish_discovery_payload(s_mqtt.light_discovery_topic, payload);
        free(payload);
        payload = NULL;
    }

#if CONFIG_OTL_PRESENCE_SENSOR
    payload = otl_mqtt_alloc_printf(
        "{"
        "\"name\":\"Presence\","
        "\"unique_id\":\"%s_occupancy\","
        "\"state_topic\":\"%s\","
        "\"device_class\":\"occupancy\","
        "\"payload_on\":\"ON\","
        "\"payload_off\":\"OFF\","
        "\"availability_topic\":\"%s\","
        "\"payload_available\":\"online\","
        "\"payload_not_available\":\"offline\","
        "%s"
        "}",
        s_mqtt.device_id,
        s_mqtt.occupancy_state_topic,
        s_mqtt.availability_topic,
        device_json);
    if (payload != NULL) {
        (void)otl_mqtt_publish_discovery_payload(s_mqtt.occupancy_discovery_topic, payload);
        free(payload);
        payload = NULL;
    }

    payload = otl_mqtt_alloc_printf(
        "{"
        "\"name\":\"Motion\","
        "\"unique_id\":\"%s_motion\","
        "\"state_topic\":\"%s\","
        "\"device_class\":\"motion\","
        "\"payload_on\":\"ON\","
        "\"payload_off\":\"OFF\","
        "\"availability_topic\":\"%s\","
        "\"payload_available\":\"online\","
        "\"payload_not_available\":\"offline\","
        "%s"
        "}",
        s_mqtt.device_id,
        s_mqtt.motion_state_topic,
        s_mqtt.availability_topic,
        device_json);
    if (payload != NULL) {
        (void)otl_mqtt_publish_discovery_payload(s_mqtt.motion_discovery_topic, payload);
        free(payload);
        payload = NULL;
    }

    payload = otl_mqtt_alloc_printf(
        "{"
        "\"name\":\"Radar Motion Distance\","
        "\"unique_id\":\"%s_motion_distance\","
        "\"state_topic\":\"%s\","
        "\"unit_of_measurement\":\"ft\","
        "\"state_class\":\"measurement\","
        "\"entity_category\":\"diagnostic\","
        "\"icon\":\"mdi:ruler\","
        "\"availability_topic\":\"%s\","
        "\"payload_available\":\"online\","
        "\"payload_not_available\":\"offline\","
        "%s"
        "}",
        s_mqtt.device_id,
        s_mqtt.motion_distance_state_topic,
        s_mqtt.availability_topic,
        device_json);
    if (payload != NULL) {
        (void)otl_mqtt_publish_discovery_payload(s_mqtt.motion_distance_discovery_topic, payload);
        free(payload);
        payload = NULL;
    }

    payload = otl_mqtt_alloc_printf(
        "{"
        "\"name\":\"Radar Presence Distance\","
        "\"unique_id\":\"%s_stationary_distance\","
        "\"state_topic\":\"%s\","
        "\"unit_of_measurement\":\"ft\","
        "\"state_class\":\"measurement\","
        "\"entity_category\":\"diagnostic\","
        "\"icon\":\"mdi:ruler\","
        "\"availability_topic\":\"%s\","
        "\"payload_available\":\"online\","
        "\"payload_not_available\":\"offline\","
        "%s"
        "}",
        s_mqtt.device_id,
        s_mqtt.stationary_distance_state_topic,
        s_mqtt.availability_topic,
        device_json);
    if (payload != NULL) {
        (void)otl_mqtt_publish_discovery_payload(s_mqtt.stationary_distance_discovery_topic, payload);
        free(payload);
        payload = NULL;
    }
#endif

    payload = otl_mqtt_alloc_printf(
        "{"
        "\"name\":\"Ambient Light\","
        "\"unique_id\":\"%s_ambient_lux\","
        "\"state_topic\":\"%s\","
        "\"device_class\":\"illuminance\","
        "\"unit_of_measurement\":\"lx\","
        "\"state_class\":\"measurement\","
        "\"availability_topic\":\"%s\","
        "\"payload_available\":\"online\","
        "\"payload_not_available\":\"offline\","
        "%s"
        "}",
        s_mqtt.device_id,
        s_mqtt.ambient_lux_state_topic,
        s_mqtt.availability_topic,
        device_json);
    if (payload != NULL) {
        (void)otl_mqtt_publish_discovery_payload(s_mqtt.ambient_lux_discovery_topic, payload);
        free(payload);
        payload = NULL;
    }

    payload = otl_mqtt_alloc_printf(
        "{"
        "\"name\":\"Temperature LED NTC\","
        "\"unique_id\":\"%s_ntc_temperature\","
        "\"state_topic\":\"%s\","
        "\"unit_of_measurement\":\"°C\","
        "\"state_class\":\"measurement\","
        "\"entity_category\":\"diagnostic\","
        "\"availability_topic\":\"%s\","
        "\"payload_available\":\"online\","
        "\"payload_not_available\":\"offline\","
        "%s"
        "}",
        s_mqtt.device_id,
        s_mqtt.ntc_temp_state_topic,
        s_mqtt.availability_topic,
        device_json);
    if (payload != NULL) {
        (void)otl_mqtt_publish_discovery_payload(s_mqtt.ntc_temp_discovery_topic, payload);
        free(payload);
        payload = NULL;
    }

    payload = otl_mqtt_alloc_printf(
        "{"
        "\"name\":\"Temperature ESP32-S3 Internal\","
        "\"unique_id\":\"%s_chip_temperature\","
        "\"state_topic\":\"%s\","
        "\"unit_of_measurement\":\"°C\","
        "\"state_class\":\"measurement\","
        "\"entity_category\":\"diagnostic\","
        "\"availability_topic\":\"%s\","
        "\"payload_available\":\"online\","
        "\"payload_not_available\":\"offline\","
        "%s"
        "}",
        s_mqtt.device_id,
        s_mqtt.chip_temp_state_topic,
        s_mqtt.availability_topic,
        device_json);
    if (payload != NULL) {
        (void)otl_mqtt_publish_discovery_payload(s_mqtt.chip_temp_discovery_topic, payload);
        free(payload);
        payload = NULL;
    }

    payload = otl_mqtt_alloc_printf(
        "{"
        "\"name\":\"Max Brightness Limit\","
        "\"unique_id\":\"%s_thermal_cap\","
        "\"state_topic\":\"%s\","
        "\"unit_of_measurement\":\"%%\","
        "\"icon\":\"mdi:brightness-percent\","
        "\"entity_category\":\"diagnostic\","
        "\"availability_topic\":\"%s\","
        "\"payload_available\":\"online\","
        "\"payload_not_available\":\"offline\","
        "%s"
        "}",
        s_mqtt.device_id,
        s_mqtt.thermal_cap_state_topic,
        s_mqtt.availability_topic,
        device_json);
    if (payload != NULL) {
        (void)otl_mqtt_publish_discovery_payload(s_mqtt.thermal_cap_discovery_topic, payload);
        free(payload);
        payload = NULL;
    }

    payload = otl_mqtt_alloc_printf(
        "{"
        "\"name\":\"Thermal Throttle\","
        "\"unique_id\":\"%s_thermal_limited\","
        "\"state_topic\":\"%s\","
        "\"device_class\":\"problem\","
        "\"payload_on\":\"ON\","
        "\"payload_off\":\"OFF\","
        "\"entity_category\":\"diagnostic\","
        "\"availability_topic\":\"%s\","
        "\"payload_available\":\"online\","
        "\"payload_not_available\":\"offline\","
        "%s"
        "}",
        s_mqtt.device_id,
        s_mqtt.thermal_limited_state_topic,
        s_mqtt.availability_topic,
        device_json);
    if (payload != NULL) {
        (void)otl_mqtt_publish_discovery_payload(s_mqtt.thermal_limited_discovery_topic, payload);
        free(payload);
        payload = NULL;
    }

    payload = otl_mqtt_alloc_printf(
        "{"
        "\"name\":\"WiFi RSSI\","
        "\"unique_id\":\"%s_wifi_rssi\","
        "\"state_topic\":\"%s\","
        "\"device_class\":\"signal_strength\","
        "\"unit_of_measurement\":\"dBm\","
        "\"entity_category\":\"diagnostic\","
        "\"availability_topic\":\"%s\","
        "\"payload_available\":\"online\","
        "\"payload_not_available\":\"offline\","
        "%s"
        "}",
        s_mqtt.device_id,
        s_mqtt.wifi_rssi_state_topic,
        s_mqtt.availability_topic,
        device_json);
    if (payload != NULL) {
        (void)otl_mqtt_publish_discovery_payload(s_mqtt.wifi_rssi_discovery_topic, payload);
        free(payload);
        payload = NULL;
    }

    payload = otl_mqtt_alloc_printf(
        "{"
        "\"name\":\"LED Thermal Limit\","
        "\"unique_id\":\"%s_led_thermal_limit\","
        "\"command_topic\":\"%s\","
        "\"state_topic\":\"%s\","
        "\"unit_of_measurement\":\"°C\","
        "\"entity_category\":\"config\","
        "\"min\":60,"
        "\"max\":110,"
        "\"step\":1,"
        "\"mode\":\"box\","
        "\"availability_topic\":\"%s\","
        "\"payload_available\":\"online\","
        "\"payload_not_available\":\"offline\","
        "%s"
        "}",
        s_mqtt.device_id,
        s_mqtt.led_thermal_limit_command_topic,
        s_mqtt.led_thermal_limit_state_topic,
        s_mqtt.availability_topic,
        device_json);
    if (payload != NULL) {
        (void)otl_mqtt_publish_discovery_payload(s_mqtt.led_thermal_limit_discovery_topic, payload);
        free(payload);
        payload = NULL;
    }

#if CONFIG_OTL_PRESENCE_SENSOR
    payload = otl_mqtt_alloc_printf(
        "{"
        "\"name\":\"Radar Motion Max Distance\","
        "\"unique_id\":\"%s_motion_max_distance\","
        "\"command_topic\":\"%s\","
        "\"state_topic\":\"%s\","
        "\"unit_of_measurement\":\"ft\","
        "\"entity_category\":\"config\","
        "\"min\":%.1f,"
        "\"max\":%.1f,"
        "\"step\":0.1,"
        "\"mode\":\"box\","
        "\"icon\":\"mdi:ruler\","
        "\"availability_topic\":\"%s\","
        "\"payload_available\":\"online\","
        "\"payload_not_available\":\"offline\","
        "%s"
        "}",
        s_mqtt.device_id,
        s_mqtt.radar_motion_max_distance_command_topic,
        s_mqtt.radar_motion_max_distance_state_topic,
        otl_mqtt_cm_to_feet(50.0f),
        otl_mqtt_cm_to_feet(600.0f),
        s_mqtt.availability_topic,
        device_json);
    if (payload != NULL) {
        (void)otl_mqtt_publish_discovery_payload(s_mqtt.radar_motion_max_distance_discovery_topic, payload);
        free(payload);
        payload = NULL;
    }

    payload = otl_mqtt_alloc_printf(
        "{"
        "\"name\":\"Radar Presence Max Distance\","
        "\"unique_id\":\"%s_stationary_max_distance\","
        "\"command_topic\":\"%s\","
        "\"state_topic\":\"%s\","
        "\"unit_of_measurement\":\"ft\","
        "\"entity_category\":\"config\","
        "\"min\":%.1f,"
        "\"max\":%.1f,"
        "\"step\":0.1,"
        "\"mode\":\"box\","
        "\"icon\":\"mdi:ruler\","
        "\"availability_topic\":\"%s\","
        "\"payload_available\":\"online\","
        "\"payload_not_available\":\"offline\","
        "%s"
        "}",
        s_mqtt.device_id,
        s_mqtt.radar_stationary_max_distance_command_topic,
        s_mqtt.radar_stationary_max_distance_state_topic,
        otl_mqtt_cm_to_feet(50.0f),
        otl_mqtt_cm_to_feet(600.0f),
        s_mqtt.availability_topic,
        device_json);
    if (payload != NULL) {
        (void)otl_mqtt_publish_discovery_payload(s_mqtt.radar_stationary_max_distance_discovery_topic, payload);
        free(payload);
        payload = NULL;
    }
#endif

#if CONFIG_OTL_CIRCADIAN_ENABLE
    payload = otl_mqtt_alloc_printf(
        "{"
        "\"name\":\"Circadian Lighting\","
        "\"unique_id\":\"%s_circadian_enabled\","
        "\"command_topic\":\"%s\","
        "\"state_topic\":\"%s\","
        "\"payload_on\":\"ON\","
        "\"payload_off\":\"OFF\","
        "\"state_on\":\"ON\","
        "\"state_off\":\"OFF\","
        "\"entity_category\":\"config\","
        "\"availability_topic\":\"%s\","
        "\"payload_available\":\"online\","
        "\"payload_not_available\":\"offline\","
        "%s"
        "}",
        s_mqtt.device_id,
        s_mqtt.circadian_enabled_command_topic,
        s_mqtt.circadian_enabled_state_topic,
        s_mqtt.availability_topic,
        device_json);
    if (payload != NULL) {
        (void)otl_mqtt_publish_discovery_payload(s_mqtt.circadian_enabled_discovery_topic, payload);
        free(payload);
        payload = NULL;
    }

    payload = otl_mqtt_alloc_printf(
        "{"
        "\"name\":\"Circadian Lighting Coolest Time\","
        "\"unique_id\":\"%s_circadian_coolest\","
        "\"command_topic\":\"%s\","
        "\"state_topic\":\"%s\","
        "\"pattern\":\"^([01][0-9]|2[0-3]):[0-5][0-9]$\","
        "\"min\":5,"
        "\"max\":5,"
        "\"mode\":\"text\","
        "\"icon\":\"mdi:clock-outline\","
        "\"entity_category\":\"config\","
        "\"availability_topic\":\"%s\","
        "\"payload_available\":\"online\","
        "\"payload_not_available\":\"offline\","
        "%s"
        "}",
        s_mqtt.device_id,
        s_mqtt.circadian_coolest_command_topic,
        s_mqtt.circadian_coolest_state_topic,
        s_mqtt.availability_topic,
        device_json);
    if (payload != NULL) {
        (void)otl_mqtt_publish_discovery_payload(s_mqtt.circadian_coolest_discovery_topic, payload);
        free(payload);
        payload = NULL;
    }

    payload = otl_mqtt_alloc_printf(
        "{"
        "\"name\":\"Circadian Lighting Warmest Time\","
        "\"unique_id\":\"%s_circadian_warmest\","
        "\"command_topic\":\"%s\","
        "\"state_topic\":\"%s\","
        "\"pattern\":\"^([01][0-9]|2[0-3]):[0-5][0-9]$\","
        "\"min\":5,"
        "\"max\":5,"
        "\"mode\":\"text\","
        "\"icon\":\"mdi:clock-outline\","
        "\"entity_category\":\"config\","
        "\"availability_topic\":\"%s\","
        "\"payload_available\":\"online\","
        "\"payload_not_available\":\"offline\","
        "%s"
        "}",
        s_mqtt.device_id,
        s_mqtt.circadian_warmest_command_topic,
        s_mqtt.circadian_warmest_state_topic,
        s_mqtt.availability_topic,
        device_json);
    if (payload != NULL) {
        (void)otl_mqtt_publish_discovery_payload(s_mqtt.circadian_warmest_discovery_topic, payload);
        free(payload);
        payload = NULL;
    }

    payload = otl_mqtt_alloc_printf(
        "{"
        "\"name\":\"Circadian Morning Ramp\","
        "\"unique_id\":\"%s_circadian_morning_ramp\","
        "\"command_topic\":\"%s\","
        "\"state_topic\":\"%s\","
        "\"min\":0,"
        "\"max\":1440,"
        "\"step\":15,"
        "\"mode\":\"box\","
        "\"unit_of_measurement\":\"min\","
        "\"icon\":\"mdi:weather-sunset-up\","
        "\"entity_category\":\"config\","
        "\"availability_topic\":\"%s\","
        "\"payload_available\":\"online\","
        "\"payload_not_available\":\"offline\","
        "%s"
        "}",
        s_mqtt.device_id,
        s_mqtt.circadian_morning_ramp_command_topic,
        s_mqtt.circadian_morning_ramp_state_topic,
        s_mqtt.availability_topic,
        device_json);
    if (payload != NULL) {
        (void)otl_mqtt_publish_discovery_payload(s_mqtt.circadian_morning_ramp_discovery_topic, payload);
        free(payload);
        payload = NULL;
    }
#endif

    payload = otl_mqtt_alloc_printf(
        "{"
        "\"name\":\"Verbose Diagnostics\","
        "\"unique_id\":\"%s_verbose_diagnostics\","
        "\"command_topic\":\"%s\","
        "\"state_topic\":\"%s\","
        "\"payload_on\":\"ON\","
        "\"payload_off\":\"OFF\","
        "\"state_on\":\"ON\","
        "\"state_off\":\"OFF\","
        "\"entity_category\":\"diagnostic\","
        "\"availability_topic\":\"%s\","
        "\"payload_available\":\"online\","
        "\"payload_not_available\":\"offline\","
        "%s"
        "}",
        s_mqtt.device_id,
        s_mqtt.verbose_diag_command_topic,
        s_mqtt.verbose_diag_state_topic,
        s_mqtt.availability_topic,
        device_json);
    if (payload != NULL) {
        (void)otl_mqtt_publish_discovery_payload(s_mqtt.verbose_diag_discovery_topic, payload);
        free(payload);
        payload = NULL;
    }

    otl_mqtt_publish_switch("Serial Status Logs",
                            "status_logs",
                            s_mqtt.status_log_command_topic,
                            s_mqtt.status_log_state_topic,
                            s_mqtt.status_log_discovery_topic,
                            "mdi:text-box-outline",
                            "diagnostic",
                            device_json);
    otl_mqtt_publish_switch("Serial Sensor Debug Logs",
                            "sensor_debug_logs",
                            s_mqtt.sensor_debug_command_topic,
                            s_mqtt.sensor_debug_state_topic,
                            s_mqtt.sensor_debug_discovery_topic,
                            "mdi:tune-variant",
                            "diagnostic",
                            device_json);
    otl_mqtt_publish_switch("Serial Touch Event Logs",
                            "touch_event_logs",
                            s_mqtt.touch_event_log_command_topic,
                            s_mqtt.touch_event_log_state_topic,
                            s_mqtt.touch_event_log_discovery_topic,
                            "mdi:gesture-tap",
                            "diagnostic",
                            device_json);
    otl_mqtt_publish_switch("Serial Touch Calibration Logs",
                            "touch_calibration_logs",
                            s_mqtt.touch_cal_log_command_topic,
                            s_mqtt.touch_cal_log_state_topic,
                            s_mqtt.touch_cal_log_discovery_topic,
                            "mdi:tune",
                            "diagnostic",
                            device_json);
    otl_mqtt_publish_switch("Serial Touch Raw Logs",
                            "touch_raw_logs",
                            s_mqtt.touch_raw_log_command_topic,
                            s_mqtt.touch_raw_log_state_topic,
                            s_mqtt.touch_raw_log_discovery_topic,
                            "mdi:waveform",
                            "diagnostic",
                            device_json);
    otl_mqtt_publish_switch("Serial PWM Duty Logs",
                            "pwm_duty_logs",
                            s_mqtt.pwm_log_command_topic,
                            s_mqtt.pwm_log_state_topic,
                            s_mqtt.pwm_log_discovery_topic,
                            "mdi:sine-wave",
                            "diagnostic",
                            device_json);
#if CONFIG_OTL_PRESENCE_SENSOR
    otl_mqtt_publish_switch("Serial Radar Status Logs",
                            "radar_status_logs",
                            s_mqtt.radar_log_command_topic,
                            s_mqtt.radar_log_state_topic,
                            s_mqtt.radar_log_discovery_topic,
                            "mdi:radar",
                            "diagnostic",
                            device_json);
    otl_mqtt_publish_switch("Radar Occupancy Light Auto-Off",
                            "occupancy_auto_off",
                            s_mqtt.occupancy_auto_off_command_topic,
                            s_mqtt.occupancy_auto_off_state_topic,
                            s_mqtt.occupancy_auto_off_discovery_topic,
                            "mdi:motion-sensor-off",
                            "config",
                            device_json);
#endif

    otl_mqtt_publish_text_sensor("Firmware Version",
                                 "firmware_version",
                                 s_mqtt.firmware_version_state_topic,
                                 s_mqtt.firmware_version_discovery_topic,
                                 "mdi:tag-text-outline",
                                 "diagnostic",
                                 device_json);
    otl_mqtt_publish_text_sensor("Firmware Built",
                                 "firmware_built",
                                 s_mqtt.firmware_built_state_topic,
                                 s_mqtt.firmware_built_discovery_topic,
                                 "mdi:calendar-clock",
                                 "diagnostic",
                                 device_json);
    otl_mqtt_publish_text_sensor("Firmware ESP-IDF",
                                 "firmware_idf",
                                 s_mqtt.firmware_idf_state_topic,
                                 s_mqtt.firmware_idf_discovery_topic,
                                 "mdi:chip",
                                 "diagnostic",
                                 device_json);
    otl_mqtt_publish_text_sensor("Firmware Target",
                                 "firmware_target",
                                 s_mqtt.firmware_target_state_topic,
                                 s_mqtt.firmware_target_discovery_topic,
                                 "mdi:memory",
                                 "diagnostic",
                                 device_json);
    otl_mqtt_publish_binary_sensor("Firmware HomeKit Support",
                                   "firmware_homekit",
                                   s_mqtt.firmware_homekit_state_topic,
                                   s_mqtt.firmware_homekit_discovery_topic,
                                   "mdi:home-automation",
                                   "diagnostic",
                                   device_json);
    otl_mqtt_publish_binary_sensor("Firmware Presence Sensor Support",
                                   "firmware_presence_sensor",
                                   s_mqtt.firmware_presence_state_topic,
                                   s_mqtt.firmware_presence_discovery_topic,
                                   "mdi:radar",
                                   "diagnostic",
                                   device_json);
    otl_mqtt_publish_binary_sensor("Firmware Circadian Lighting Support",
                                   "firmware_circadian",
                                   s_mqtt.firmware_circadian_state_topic,
                                   s_mqtt.firmware_circadian_discovery_topic,
                                   "mdi:theme-light-dark",
                                   "diagnostic",
                                   device_json);

    payload = otl_mqtt_alloc_printf(
        "{"
        "\"name\":\"Last Event\","
        "\"unique_id\":\"%s_last_event\","
        "\"state_topic\":\"%s\","
        "\"icon\":\"mdi:text-box-outline\","
        "\"entity_category\":\"diagnostic\","
        "\"availability_topic\":\"%s\","
        "\"payload_available\":\"online\","
        "\"payload_not_available\":\"offline\","
        "%s"
        "}",
        s_mqtt.device_id,
        s_mqtt.last_event_state_topic,
        s_mqtt.availability_topic,
        device_json);
    if (payload != NULL) {
        (void)otl_mqtt_publish_discovery_payload(s_mqtt.last_event_discovery_topic, payload);
        free(payload);
        payload = NULL;
    }

    free(device_json);
}

static void otl_mqtt_republish_settings_after_invalid_write(void)
{
    otl_runtime_settings_notify_current(OTL_CHANGE_SOURCE_SYSTEM);
}

static bool otl_mqtt_handle_bool_setting_command(esp_mqtt_event_handle_t event,
                                                 const char *topic,
                                                 otl_mqtt_bool_setting_setter_fn setter,
                                                 const char *rejected_message,
                                                 const char *failed_message)
{
    bool enabled = false;

    if (!otl_mqtt_topic_matches(event, topic)) {
        return false;
    }

    if (!otl_mqtt_parse_on_off_payload(event, &enabled)) {
        otl_event_emit(OTL_EVENT_LEVEL_WARNING, "mqtt", rejected_message);
        otl_mqtt_republish_settings_after_invalid_write();
        return true;
    }

    if (setter(enabled, OTL_CHANGE_SOURCE_MQTT) != ESP_OK) {
        otl_event_emit(OTL_EVENT_LEVEL_WARNING, "mqtt", failed_message);
        otl_mqtt_republish_settings_after_invalid_write();
    }
    return true;
}

static void otl_mqtt_handle_command(esp_mqtt_event_handle_t event)
{
    if (otl_mqtt_topic_matches(event, s_mqtt.light_command_topic)) {
        bool power_on = false;
        if (!otl_mqtt_parse_on_off_payload(event, &power_on)) {
            return;
        }

        (void)otl_state_apply_light_update(&(otl_light_update_t){
            .set_power = true,
            .power_on = power_on,
        }, OTL_CHANGE_SOURCE_MQTT);
        return;
    }

    if (otl_mqtt_topic_matches(event, s_mqtt.brightness_command_topic)) {
        int brightness_ha = 0;
        if (!otl_mqtt_parse_int_payload(event, &brightness_ha)) {
            return;
        }

        if (brightness_ha <= 0) {
            (void)otl_state_apply_light_update(&(otl_light_update_t){
                .set_power = true,
                .power_on = false,
            }, OTL_CHANGE_SOURCE_MQTT);
            return;
        }

        (void)otl_state_apply_light_update(&(otl_light_update_t){
            .set_power = true,
            .power_on = true,
            .set_brightness = true,
            .force_on_with_brightness = true,
            .brightness_percent = otl_mqtt_brightness_from_ha(brightness_ha),
        }, OTL_CHANGE_SOURCE_MQTT);
        return;
    }

    if (otl_mqtt_topic_matches(event, s_mqtt.color_temp_command_topic)) {
        int mired = 0;
        if (!otl_mqtt_parse_int_payload(event, &mired)) {
            return;
        }

        (void)otl_state_apply_light_update(&(otl_light_update_t){
            .set_temp_ratio = true,
            .temp_ratio = otl_mqtt_mired_to_temp_ratio(mired),
        }, OTL_CHANGE_SOURCE_MQTT);
        return;
    }

    if (otl_mqtt_handle_bool_setting_command(event,
                                             s_mqtt.status_log_command_topic,
                                             otl_runtime_set_status_logging_enabled,
                                             "Rejected status log write",
                                             "Failed to update status logs")) {
        return;
    }

    if (otl_mqtt_handle_bool_setting_command(event,
                                             s_mqtt.sensor_debug_command_topic,
                                             otl_runtime_set_sensor_debug_logging_enabled,
                                             "Rejected sensor debug log write",
                                             "Failed to update sensor debug logs")) {
        return;
    }

    if (otl_mqtt_handle_bool_setting_command(event,
                                             s_mqtt.touch_event_log_command_topic,
                                             otl_runtime_set_touch_event_logging_enabled,
                                             "Rejected touch event log write",
                                             "Failed to update touch event logs")) {
        return;
    }

    if (otl_mqtt_handle_bool_setting_command(event,
                                             s_mqtt.touch_cal_log_command_topic,
                                             otl_runtime_set_touch_calibration_logging_enabled,
                                             "Rejected touch calibration log write",
                                             "Failed to update touch calibration logs")) {
        return;
    }

    if (otl_mqtt_handle_bool_setting_command(event,
                                             s_mqtt.touch_raw_log_command_topic,
                                             otl_runtime_set_touch_raw_logging_enabled,
                                             "Rejected touch raw log write",
                                             "Failed to update touch raw logs")) {
        return;
    }

    if (otl_mqtt_handle_bool_setting_command(event,
                                             s_mqtt.pwm_log_command_topic,
                                             otl_runtime_set_pwm_duty_logging_enabled,
                                             "Rejected PWM log write",
                                             "Failed to update PWM logs")) {
        return;
    }

#if CONFIG_OTL_PRESENCE_SENSOR
    if (otl_mqtt_handle_bool_setting_command(event,
                                             s_mqtt.radar_log_command_topic,
                                             otl_runtime_set_radar_status_logging_enabled,
                                             "Rejected radar log write",
                                             "Failed to update radar logs")) {
        return;
    }

    if (otl_mqtt_handle_bool_setting_command(event,
                                             s_mqtt.occupancy_auto_off_command_topic,
                                             otl_runtime_set_occupancy_auto_off_enabled,
                                             "Rejected occupancy auto-off write",
                                             "Failed to update occupancy auto-off")) {
        return;
    }
#endif

#if CONFIG_OTL_CIRCADIAN_ENABLE
    if (otl_mqtt_topic_matches(event, s_mqtt.circadian_enabled_command_topic)) {
        bool enabled = false;
        if (!otl_mqtt_parse_on_off_payload(event, &enabled)) {
            otl_event_emit(OTL_EVENT_LEVEL_WARNING, "mqtt", "Rejected circadian enable write");
            otl_mqtt_republish_settings_after_invalid_write();
            return;
        }
        if (otl_runtime_set_circadian_enabled(enabled, OTL_CHANGE_SOURCE_MQTT) != ESP_OK) {
            otl_event_emit(OTL_EVENT_LEVEL_WARNING, "mqtt", "Failed to update circadian enable");
            otl_mqtt_republish_settings_after_invalid_write();
        }
        return;
    }

    if (otl_mqtt_topic_matches(event, s_mqtt.circadian_coolest_command_topic)) {
        char payload[OTL_RUNTIME_TIME_STR_LEN + 8] = {0};
        if (!otl_mqtt_copy_payload(event, payload, sizeof(payload))) {
            otl_event_emit(OTL_EVENT_LEVEL_WARNING, "mqtt", "Rejected coolest time write");
            otl_mqtt_republish_settings_after_invalid_write();
            return;
        }
        otl_mqtt_trim(payload);
        if (otl_runtime_set_circadian_coolest_time(payload, OTL_CHANGE_SOURCE_MQTT) != ESP_OK) {
            otl_event_emit(OTL_EVENT_LEVEL_WARNING, "mqtt", "Rejected coolest time write");
            otl_mqtt_republish_settings_after_invalid_write();
        }
        return;
    }

    if (otl_mqtt_topic_matches(event, s_mqtt.circadian_warmest_command_topic)) {
        char payload[OTL_RUNTIME_TIME_STR_LEN + 8] = {0};
        if (!otl_mqtt_copy_payload(event, payload, sizeof(payload))) {
            otl_event_emit(OTL_EVENT_LEVEL_WARNING, "mqtt", "Rejected warmest time write");
            otl_mqtt_republish_settings_after_invalid_write();
            return;
        }
        otl_mqtt_trim(payload);
        if (otl_runtime_set_circadian_warmest_time(payload, OTL_CHANGE_SOURCE_MQTT) != ESP_OK) {
            otl_event_emit(OTL_EVENT_LEVEL_WARNING, "mqtt", "Rejected warmest time write");
            otl_mqtt_republish_settings_after_invalid_write();
        }
        return;
    }

    if (otl_mqtt_topic_matches(event, s_mqtt.circadian_morning_ramp_command_topic)) {
        int minutes = 0;
        if (!otl_mqtt_parse_int_payload(event, &minutes)) {
            otl_event_emit(OTL_EVENT_LEVEL_WARNING, "mqtt", "Rejected circadian morning ramp write");
            otl_mqtt_republish_settings_after_invalid_write();
            return;
        }
        if (otl_runtime_set_circadian_morning_ramp_minutes(minutes, OTL_CHANGE_SOURCE_MQTT) != ESP_OK) {
            otl_event_emit(OTL_EVENT_LEVEL_WARNING, "mqtt", "Rejected circadian morning ramp write");
            otl_mqtt_republish_settings_after_invalid_write();
        }
        return;
    }
#endif

    if (otl_mqtt_topic_matches(event, s_mqtt.led_thermal_limit_command_topic)) {
        float limit_c = 0.0f;
        if (!otl_mqtt_parse_float_payload(event, &limit_c)) {
            otl_event_emit(OTL_EVENT_LEVEL_WARNING, "mqtt", "Rejected LED thermal limit write");
            otl_mqtt_republish_settings_after_invalid_write();
            return;
        }
        if (otl_runtime_set_led_thermal_limit_c(limit_c, OTL_CHANGE_SOURCE_MQTT) != ESP_OK) {
            otl_event_emit(OTL_EVENT_LEVEL_WARNING, "mqtt", "Rejected LED thermal limit write");
            otl_mqtt_republish_settings_after_invalid_write();
        }
        return;
    }

#if CONFIG_OTL_PRESENCE_SENSOR
    if (otl_mqtt_topic_matches(event, s_mqtt.radar_motion_max_distance_command_topic)) {
        float limit_ft = 0.0f;
        if (!otl_mqtt_parse_float_payload(event, &limit_ft)) {
            otl_event_emit(OTL_EVENT_LEVEL_WARNING, "mqtt", "Rejected motion max distance write");
            otl_mqtt_republish_settings_after_invalid_write();
            return;
        }
        if (otl_runtime_set_radar_motion_max_distance_cm(otl_mqtt_feet_to_cm(limit_ft), OTL_CHANGE_SOURCE_MQTT) != ESP_OK) {
            otl_event_emit(OTL_EVENT_LEVEL_WARNING, "mqtt", "Rejected motion max distance write");
            otl_mqtt_republish_settings_after_invalid_write();
        }
        return;
    }

    if (otl_mqtt_topic_matches(event, s_mqtt.radar_stationary_max_distance_command_topic)) {
        float limit_ft = 0.0f;
        if (!otl_mqtt_parse_float_payload(event, &limit_ft)) {
            otl_event_emit(OTL_EVENT_LEVEL_WARNING, "mqtt", "Rejected stationary max distance write");
            otl_mqtt_republish_settings_after_invalid_write();
            return;
        }
        if (otl_runtime_set_radar_stationary_max_distance_cm(otl_mqtt_feet_to_cm(limit_ft), OTL_CHANGE_SOURCE_MQTT) != ESP_OK) {
            otl_event_emit(OTL_EVENT_LEVEL_WARNING, "mqtt", "Rejected stationary max distance write");
            otl_mqtt_republish_settings_after_invalid_write();
        }
        return;
    }
#endif

    if (otl_mqtt_topic_matches(event, s_mqtt.verbose_diag_command_topic)) {
        bool enabled = false;
        if (!otl_mqtt_parse_on_off_payload(event, &enabled)) {
            otl_event_emit(OTL_EVENT_LEVEL_WARNING, "mqtt", "Rejected verbose diagnostics write");
            otl_mqtt_republish_settings_after_invalid_write();
            return;
        }
        if (otl_runtime_set_verbose_diagnostics(enabled, OTL_CHANGE_SOURCE_MQTT) != ESP_OK) {
            otl_event_emit(OTL_EVENT_LEVEL_WARNING, "mqtt", "Failed to update verbose diagnostics");
            otl_mqtt_republish_settings_after_invalid_write();
        }
    }
}

static void otl_mqtt_state_listener(const otl_public_state_t *state,
                                    otl_change_source_t source,
                                    void *ctx)
{
    (void)source;
    (void)ctx;
    otl_mqtt_publish_state(state);
}

static void otl_mqtt_telemetry_listener(const otl_telemetry_t *telemetry, void *ctx)
{
    (void)ctx;
    otl_mqtt_publish_telemetry(telemetry);
}

static void otl_mqtt_settings_listener(const otl_runtime_settings_t *settings,
                                       otl_change_source_t source,
                                       void *ctx)
{
    (void)source;
    (void)ctx;
    otl_mqtt_publish_settings(settings);
}

static void otl_mqtt_event_listener(const otl_runtime_event_t *event, void *ctx)
{
    (void)ctx;
    otl_mqtt_publish_event(event);
}

static esp_err_t otl_mqtt_build_topics(void)
{
    uint8_t mac[6] = {0};

    ESP_ERROR_CHECK(esp_read_mac(mac, ESP_MAC_WIFI_STA));

    snprintf(s_mqtt.device_id,
             sizeof(s_mqtt.device_id),
             "open_task_light_%02x%02x%02x%02x%02x%02x",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    ESP_RETURN_ON_ERROR(otl_mqtt_format_topic(s_mqtt.topic_root,
                                              sizeof(s_mqtt.topic_root),
                                              "%s/%s",
                                              CONFIG_OTL_MQTT_TOPIC_BASE,
                                              s_mqtt.device_id),
                        TAG,
                        "MQTT topic root too long");
    ESP_RETURN_ON_ERROR(otl_mqtt_format_topic(s_mqtt.availability_topic,
                                              sizeof(s_mqtt.availability_topic),
                                              "%s/availability",
                                              s_mqtt.topic_root),
                        TAG,
                        "MQTT availability topic too long");
    ESP_RETURN_ON_ERROR(otl_mqtt_format_topic(s_mqtt.event_stream_topic,
                                              sizeof(s_mqtt.event_stream_topic),
                                              "%s/events",
                                              s_mqtt.topic_root),
                        TAG,
                        "MQTT event topic too long");
    ESP_RETURN_ON_ERROR(otl_mqtt_format_topic(s_mqtt.light_state_topic,
                                              sizeof(s_mqtt.light_state_topic),
                                              "%s/light/state",
                                              s_mqtt.topic_root),
                        TAG,
                        "MQTT light state topic too long");
    ESP_RETURN_ON_ERROR(otl_mqtt_format_topic(s_mqtt.light_command_topic,
                                              sizeof(s_mqtt.light_command_topic),
                                              "%s/light/set",
                                              s_mqtt.topic_root),
                        TAG,
                        "MQTT light command topic too long");
    ESP_RETURN_ON_ERROR(otl_mqtt_format_topic(s_mqtt.brightness_state_topic,
                                              sizeof(s_mqtt.brightness_state_topic),
                                              "%s/light/brightness/state",
                                              s_mqtt.topic_root),
                        TAG,
                        "MQTT brightness state topic too long");
    ESP_RETURN_ON_ERROR(otl_mqtt_format_topic(s_mqtt.brightness_command_topic,
                                              sizeof(s_mqtt.brightness_command_topic),
                                              "%s/light/brightness/set",
                                              s_mqtt.topic_root),
                        TAG,
                        "MQTT brightness command topic too long");
    ESP_RETURN_ON_ERROR(otl_mqtt_format_topic(s_mqtt.color_temp_state_topic,
                                              sizeof(s_mqtt.color_temp_state_topic),
                                              "%s/light/color_temp/state",
                                              s_mqtt.topic_root),
                        TAG,
                        "MQTT color temperature state topic too long");
    ESP_RETURN_ON_ERROR(otl_mqtt_format_topic(s_mqtt.color_temp_command_topic,
                                              sizeof(s_mqtt.color_temp_command_topic),
                                              "%s/light/color_temp/set",
                                              s_mqtt.topic_root),
                        TAG,
                        "MQTT color temperature command topic too long");
    ESP_RETURN_ON_ERROR(otl_mqtt_format_topic(s_mqtt.light_discovery_topic,
                                              sizeof(s_mqtt.light_discovery_topic),
                                              "%s/light/%s_light/config",
                                              CONFIG_OTL_MQTT_DISCOVERY_PREFIX,
                                              s_mqtt.device_id),
                        TAG,
                        "MQTT light discovery topic too long");
#if CONFIG_OTL_PRESENCE_SENSOR
    ESP_RETURN_ON_ERROR(otl_mqtt_format_topic(s_mqtt.occupancy_state_topic,
                                              sizeof(s_mqtt.occupancy_state_topic),
                                              "%s/occupancy/state",
                                              s_mqtt.topic_root),
                        TAG,
                        "MQTT occupancy state topic too long");
    ESP_RETURN_ON_ERROR(otl_mqtt_format_topic(s_mqtt.occupancy_discovery_topic,
                                              sizeof(s_mqtt.occupancy_discovery_topic),
                                              "%s/binary_sensor/%s_occupancy/config",
                                              CONFIG_OTL_MQTT_DISCOVERY_PREFIX,
                                              s_mqtt.device_id),
                        TAG,
                        "MQTT occupancy discovery topic too long");
    ESP_RETURN_ON_ERROR(otl_mqtt_format_topic(s_mqtt.motion_state_topic,
                                              sizeof(s_mqtt.motion_state_topic),
                                              "%s/motion/state",
                                              s_mqtt.topic_root),
                        TAG,
                        "MQTT motion state topic too long");
    ESP_RETURN_ON_ERROR(otl_mqtt_format_topic(s_mqtt.motion_discovery_topic,
                                              sizeof(s_mqtt.motion_discovery_topic),
                                              "%s/binary_sensor/%s_motion/config",
                                              CONFIG_OTL_MQTT_DISCOVERY_PREFIX,
                                              s_mqtt.device_id),
                        TAG,
                        "MQTT motion discovery topic too long");
    ESP_RETURN_ON_ERROR(otl_mqtt_format_topic(s_mqtt.motion_distance_state_topic,
                                              sizeof(s_mqtt.motion_distance_state_topic),
                                              "%s/radar/motion_distance/state",
                                              s_mqtt.topic_root),
                        TAG,
                        "MQTT motion distance state topic too long");
    ESP_RETURN_ON_ERROR(otl_mqtt_format_topic(s_mqtt.motion_distance_discovery_topic,
                                              sizeof(s_mqtt.motion_distance_discovery_topic),
                                              "%s/sensor/%s_motion_distance/config",
                                              CONFIG_OTL_MQTT_DISCOVERY_PREFIX,
                                              s_mqtt.device_id),
                        TAG,
                        "MQTT motion distance discovery topic too long");
    ESP_RETURN_ON_ERROR(otl_mqtt_format_topic(s_mqtt.stationary_distance_state_topic,
                                              sizeof(s_mqtt.stationary_distance_state_topic),
                                              "%s/radar/stationary_distance/state",
                                              s_mqtt.topic_root),
                        TAG,
                        "MQTT stationary distance state topic too long");
    ESP_RETURN_ON_ERROR(otl_mqtt_format_topic(s_mqtt.stationary_distance_discovery_topic,
                                              sizeof(s_mqtt.stationary_distance_discovery_topic),
                                              "%s/sensor/%s_stationary_distance/config",
                                              CONFIG_OTL_MQTT_DISCOVERY_PREFIX,
                                              s_mqtt.device_id),
                        TAG,
                        "MQTT stationary distance discovery topic too long");
#endif
    ESP_RETURN_ON_ERROR(otl_mqtt_format_topic(s_mqtt.ambient_lux_state_topic,
                                              sizeof(s_mqtt.ambient_lux_state_topic),
                                              "%s/sensors/ambient_lux/state",
                                              s_mqtt.topic_root),
                        TAG,
                        "MQTT ambient lux state topic too long");
    ESP_RETURN_ON_ERROR(otl_mqtt_format_topic(s_mqtt.ambient_lux_discovery_topic,
                                              sizeof(s_mqtt.ambient_lux_discovery_topic),
                                              "%s/sensor/%s_ambient_lux/config",
                                              CONFIG_OTL_MQTT_DISCOVERY_PREFIX,
                                              s_mqtt.device_id),
                        TAG,
                        "MQTT ambient lux discovery topic too long");
    ESP_RETURN_ON_ERROR(otl_mqtt_format_topic(s_mqtt.ntc_temp_state_topic,
                                              sizeof(s_mqtt.ntc_temp_state_topic),
                                              "%s/sensors/ntc_temp/state",
                                              s_mqtt.topic_root),
                        TAG,
                        "MQTT NTC state topic too long");
    ESP_RETURN_ON_ERROR(otl_mqtt_format_topic(s_mqtt.ntc_temp_discovery_topic,
                                              sizeof(s_mqtt.ntc_temp_discovery_topic),
                                              "%s/sensor/%s_ntc_temperature/config",
                                              CONFIG_OTL_MQTT_DISCOVERY_PREFIX,
                                              s_mqtt.device_id),
                        TAG,
                        "MQTT NTC discovery topic too long");
    ESP_RETURN_ON_ERROR(otl_mqtt_format_topic(s_mqtt.chip_temp_state_topic,
                                              sizeof(s_mqtt.chip_temp_state_topic),
                                              "%s/sensors/chip_temp/state",
                                              s_mqtt.topic_root),
                        TAG,
                        "MQTT chip temp state topic too long");
    ESP_RETURN_ON_ERROR(otl_mqtt_format_topic(s_mqtt.chip_temp_discovery_topic,
                                              sizeof(s_mqtt.chip_temp_discovery_topic),
                                              "%s/sensor/%s_chip_temperature/config",
                                              CONFIG_OTL_MQTT_DISCOVERY_PREFIX,
                                              s_mqtt.device_id),
                        TAG,
                        "MQTT chip temp discovery topic too long");
    ESP_RETURN_ON_ERROR(otl_mqtt_format_topic(s_mqtt.thermal_cap_state_topic,
                                              sizeof(s_mqtt.thermal_cap_state_topic),
                                              "%s/sensors/thermal_cap/state",
                                              s_mqtt.topic_root),
                        TAG,
                        "MQTT thermal cap state topic too long");
    ESP_RETURN_ON_ERROR(otl_mqtt_format_topic(s_mqtt.thermal_cap_discovery_topic,
                                              sizeof(s_mqtt.thermal_cap_discovery_topic),
                                              "%s/sensor/%s_thermal_cap/config",
                                              CONFIG_OTL_MQTT_DISCOVERY_PREFIX,
                                              s_mqtt.device_id),
                        TAG,
                        "MQTT thermal cap discovery topic too long");
    ESP_RETURN_ON_ERROR(otl_mqtt_format_topic(s_mqtt.thermal_limited_state_topic,
                                              sizeof(s_mqtt.thermal_limited_state_topic),
                                              "%s/sensors/thermal_limited/state",
                                              s_mqtt.topic_root),
                        TAG,
                        "MQTT thermal limited state topic too long");
    ESP_RETURN_ON_ERROR(otl_mqtt_format_topic(s_mqtt.thermal_limited_discovery_topic,
                                              sizeof(s_mqtt.thermal_limited_discovery_topic),
                                              "%s/binary_sensor/%s_thermal_limited/config",
                                              CONFIG_OTL_MQTT_DISCOVERY_PREFIX,
                                              s_mqtt.device_id),
                        TAG,
                        "MQTT thermal limited discovery topic too long");
    ESP_RETURN_ON_ERROR(otl_mqtt_format_topic(s_mqtt.wifi_rssi_state_topic,
                                              sizeof(s_mqtt.wifi_rssi_state_topic),
                                              "%s/sensors/wifi_rssi/state",
                                              s_mqtt.topic_root),
                        TAG,
                        "MQTT RSSI state topic too long");
    ESP_RETURN_ON_ERROR(otl_mqtt_format_topic(s_mqtt.wifi_rssi_discovery_topic,
                                              sizeof(s_mqtt.wifi_rssi_discovery_topic),
                                              "%s/sensor/%s_wifi_rssi/config",
                                              CONFIG_OTL_MQTT_DISCOVERY_PREFIX,
                                              s_mqtt.device_id),
                        TAG,
                        "MQTT RSSI discovery topic too long");
    ESP_RETURN_ON_ERROR(otl_mqtt_format_topic(s_mqtt.led_thermal_limit_state_topic,
                                              sizeof(s_mqtt.led_thermal_limit_state_topic),
                                              "%s/thermal/led_limit/state",
                                              s_mqtt.topic_root),
                        TAG,
                        "MQTT LED thermal limit state topic too long");
    ESP_RETURN_ON_ERROR(otl_mqtt_format_topic(s_mqtt.led_thermal_limit_command_topic,
                                              sizeof(s_mqtt.led_thermal_limit_command_topic),
                                              "%s/thermal/led_limit/set",
                                              s_mqtt.topic_root),
                        TAG,
                        "MQTT LED thermal limit command topic too long");
    ESP_RETURN_ON_ERROR(otl_mqtt_format_topic(s_mqtt.led_thermal_limit_discovery_topic,
                                              sizeof(s_mqtt.led_thermal_limit_discovery_topic),
                                              "%s/number/%s_led_thermal_limit/config",
                                              CONFIG_OTL_MQTT_DISCOVERY_PREFIX,
                                              s_mqtt.device_id),
                        TAG,
                        "MQTT LED thermal limit discovery topic too long");
#if CONFIG_OTL_PRESENCE_SENSOR
    ESP_RETURN_ON_ERROR(otl_mqtt_format_topic(s_mqtt.radar_motion_max_distance_state_topic,
                                              sizeof(s_mqtt.radar_motion_max_distance_state_topic),
                                              "%s/radar/motion_max_distance/state",
                                              s_mqtt.topic_root),
                        TAG,
                        "MQTT motion max distance state topic too long");
    ESP_RETURN_ON_ERROR(otl_mqtt_format_topic(s_mqtt.radar_motion_max_distance_command_topic,
                                              sizeof(s_mqtt.radar_motion_max_distance_command_topic),
                                              "%s/radar/motion_max_distance/set",
                                              s_mqtt.topic_root),
                        TAG,
                        "MQTT motion max distance command topic too long");
    ESP_RETURN_ON_ERROR(otl_mqtt_format_topic(s_mqtt.radar_motion_max_distance_discovery_topic,
                                              sizeof(s_mqtt.radar_motion_max_distance_discovery_topic),
                                              "%s/number/%s_motion_max_distance/config",
                                              CONFIG_OTL_MQTT_DISCOVERY_PREFIX,
                                              s_mqtt.device_id),
                        TAG,
                        "MQTT motion max distance discovery topic too long");
    ESP_RETURN_ON_ERROR(otl_mqtt_format_topic(s_mqtt.radar_stationary_max_distance_state_topic,
                                              sizeof(s_mqtt.radar_stationary_max_distance_state_topic),
                                              "%s/radar/stationary_max_distance/state",
                                              s_mqtt.topic_root),
                        TAG,
                        "MQTT stationary max distance state topic too long");
    ESP_RETURN_ON_ERROR(otl_mqtt_format_topic(s_mqtt.radar_stationary_max_distance_command_topic,
                                              sizeof(s_mqtt.radar_stationary_max_distance_command_topic),
                                              "%s/radar/stationary_max_distance/set",
                                              s_mqtt.topic_root),
                        TAG,
                        "MQTT stationary max distance command topic too long");
    ESP_RETURN_ON_ERROR(otl_mqtt_format_topic(s_mqtt.radar_stationary_max_distance_discovery_topic,
                                              sizeof(s_mqtt.radar_stationary_max_distance_discovery_topic),
                                              "%s/number/%s_stationary_max_distance/config",
                                              CONFIG_OTL_MQTT_DISCOVERY_PREFIX,
                                              s_mqtt.device_id),
                        TAG,
                        "MQTT stationary max distance discovery topic too long");
#endif
#if CONFIG_OTL_CIRCADIAN_ENABLE
    ESP_RETURN_ON_ERROR(otl_mqtt_format_topic(s_mqtt.circadian_enabled_state_topic,
                                              sizeof(s_mqtt.circadian_enabled_state_topic),
                                              "%s/circadian/enabled/state",
                                              s_mqtt.topic_root),
                        TAG,
                        "MQTT circadian enabled state topic too long");
    ESP_RETURN_ON_ERROR(otl_mqtt_format_topic(s_mqtt.circadian_enabled_command_topic,
                                              sizeof(s_mqtt.circadian_enabled_command_topic),
                                              "%s/circadian/enabled/set",
                                              s_mqtt.topic_root),
                        TAG,
                        "MQTT circadian enabled command topic too long");
    ESP_RETURN_ON_ERROR(otl_mqtt_format_topic(s_mqtt.circadian_enabled_discovery_topic,
                                              sizeof(s_mqtt.circadian_enabled_discovery_topic),
                                              "%s/switch/%s_circadian_enabled/config",
                                              CONFIG_OTL_MQTT_DISCOVERY_PREFIX,
                                              s_mqtt.device_id),
                        TAG,
                        "MQTT circadian enabled discovery topic too long");
    ESP_RETURN_ON_ERROR(otl_mqtt_format_topic(s_mqtt.circadian_coolest_state_topic,
                                              sizeof(s_mqtt.circadian_coolest_state_topic),
                                              "%s/circadian/coolest/state",
                                              s_mqtt.topic_root),
                        TAG,
                        "MQTT coolest time state topic too long");
    ESP_RETURN_ON_ERROR(otl_mqtt_format_topic(s_mqtt.circadian_coolest_command_topic,
                                              sizeof(s_mqtt.circadian_coolest_command_topic),
                                              "%s/circadian/coolest/set",
                                              s_mqtt.topic_root),
                        TAG,
                        "MQTT coolest time command topic too long");
    ESP_RETURN_ON_ERROR(otl_mqtt_format_topic(s_mqtt.circadian_coolest_discovery_topic,
                                              sizeof(s_mqtt.circadian_coolest_discovery_topic),
                                              "%s/text/%s_circadian_coolest/config",
                                              CONFIG_OTL_MQTT_DISCOVERY_PREFIX,
                                              s_mqtt.device_id),
                        TAG,
                        "MQTT coolest time discovery topic too long");
    ESP_RETURN_ON_ERROR(otl_mqtt_format_topic(s_mqtt.circadian_warmest_state_topic,
                                              sizeof(s_mqtt.circadian_warmest_state_topic),
                                              "%s/circadian/warmest/state",
                                              s_mqtt.topic_root),
                        TAG,
                        "MQTT warmest time state topic too long");
    ESP_RETURN_ON_ERROR(otl_mqtt_format_topic(s_mqtt.circadian_warmest_command_topic,
                                              sizeof(s_mqtt.circadian_warmest_command_topic),
                                              "%s/circadian/warmest/set",
                                              s_mqtt.topic_root),
                        TAG,
                        "MQTT warmest time command topic too long");
    ESP_RETURN_ON_ERROR(otl_mqtt_format_topic(s_mqtt.circadian_warmest_discovery_topic,
                                              sizeof(s_mqtt.circadian_warmest_discovery_topic),
                                              "%s/text/%s_circadian_warmest/config",
                                              CONFIG_OTL_MQTT_DISCOVERY_PREFIX,
                                              s_mqtt.device_id),
                        TAG,
                        "MQTT warmest time discovery topic too long");
    ESP_RETURN_ON_ERROR(otl_mqtt_format_topic(s_mqtt.circadian_morning_ramp_state_topic,
                                              sizeof(s_mqtt.circadian_morning_ramp_state_topic),
                                              "%s/circadian/morning_ramp/state",
                                              s_mqtt.topic_root),
                        TAG,
                        "MQTT morning ramp state topic too long");
    ESP_RETURN_ON_ERROR(otl_mqtt_format_topic(s_mqtt.circadian_morning_ramp_command_topic,
                                              sizeof(s_mqtt.circadian_morning_ramp_command_topic),
                                              "%s/circadian/morning_ramp/set",
                                              s_mqtt.topic_root),
                        TAG,
                        "MQTT morning ramp command topic too long");
    ESP_RETURN_ON_ERROR(otl_mqtt_format_topic(s_mqtt.circadian_morning_ramp_discovery_topic,
                                              sizeof(s_mqtt.circadian_morning_ramp_discovery_topic),
                                              "%s/number/%s_circadian_morning_ramp/config",
                                              CONFIG_OTL_MQTT_DISCOVERY_PREFIX,
                                              s_mqtt.device_id),
                        TAG,
                        "MQTT morning ramp discovery topic too long");
#endif
    ESP_RETURN_ON_ERROR(otl_mqtt_format_topic(s_mqtt.verbose_diag_state_topic,
                                              sizeof(s_mqtt.verbose_diag_state_topic),
                                              "%s/diagnostics/verbose/state",
                                              s_mqtt.topic_root),
                        TAG,
                        "MQTT verbose diagnostics state topic too long");
    ESP_RETURN_ON_ERROR(otl_mqtt_format_topic(s_mqtt.verbose_diag_command_topic,
                                              sizeof(s_mqtt.verbose_diag_command_topic),
                                              "%s/diagnostics/verbose/set",
                                              s_mqtt.topic_root),
                        TAG,
                        "MQTT verbose diagnostics command topic too long");
    ESP_RETURN_ON_ERROR(otl_mqtt_format_topic(s_mqtt.verbose_diag_discovery_topic,
                                              sizeof(s_mqtt.verbose_diag_discovery_topic),
                                              "%s/switch/%s_verbose_diagnostics/config",
                                              CONFIG_OTL_MQTT_DISCOVERY_PREFIX,
                                              s_mqtt.device_id),
                        TAG,
                        "MQTT verbose diagnostics discovery topic too long");
    ESP_RETURN_ON_ERROR(otl_mqtt_format_topic(s_mqtt.status_log_state_topic,
                                              sizeof(s_mqtt.status_log_state_topic),
                                              "%s/logging/status/state",
                                              s_mqtt.topic_root),
                        TAG,
                        "MQTT status log state topic too long");
    ESP_RETURN_ON_ERROR(otl_mqtt_format_topic(s_mqtt.status_log_command_topic,
                                              sizeof(s_mqtt.status_log_command_topic),
                                              "%s/logging/status/set",
                                              s_mqtt.topic_root),
                        TAG,
                        "MQTT status log command topic too long");
    ESP_RETURN_ON_ERROR(otl_mqtt_format_topic(s_mqtt.status_log_discovery_topic,
                                              sizeof(s_mqtt.status_log_discovery_topic),
                                              "%s/switch/%s_status_logs/config",
                                              CONFIG_OTL_MQTT_DISCOVERY_PREFIX,
                                              s_mqtt.device_id),
                        TAG,
                        "MQTT status log discovery topic too long");
    ESP_RETURN_ON_ERROR(otl_mqtt_format_topic(s_mqtt.sensor_debug_state_topic,
                                              sizeof(s_mqtt.sensor_debug_state_topic),
                                              "%s/logging/sensor_debug/state",
                                              s_mqtt.topic_root),
                        TAG,
                        "MQTT sensor debug state topic too long");
    ESP_RETURN_ON_ERROR(otl_mqtt_format_topic(s_mqtt.sensor_debug_command_topic,
                                              sizeof(s_mqtt.sensor_debug_command_topic),
                                              "%s/logging/sensor_debug/set",
                                              s_mqtt.topic_root),
                        TAG,
                        "MQTT sensor debug command topic too long");
    ESP_RETURN_ON_ERROR(otl_mqtt_format_topic(s_mqtt.sensor_debug_discovery_topic,
                                              sizeof(s_mqtt.sensor_debug_discovery_topic),
                                              "%s/switch/%s_sensor_debug_logs/config",
                                              CONFIG_OTL_MQTT_DISCOVERY_PREFIX,
                                              s_mqtt.device_id),
                        TAG,
                        "MQTT sensor debug discovery topic too long");
    ESP_RETURN_ON_ERROR(otl_mqtt_format_topic(s_mqtt.touch_event_log_state_topic,
                                              sizeof(s_mqtt.touch_event_log_state_topic),
                                              "%s/logging/touch_events/state",
                                              s_mqtt.topic_root),
                        TAG,
                        "MQTT touch event log state topic too long");
    ESP_RETURN_ON_ERROR(otl_mqtt_format_topic(s_mqtt.touch_event_log_command_topic,
                                              sizeof(s_mqtt.touch_event_log_command_topic),
                                              "%s/logging/touch_events/set",
                                              s_mqtt.topic_root),
                        TAG,
                        "MQTT touch event log command topic too long");
    ESP_RETURN_ON_ERROR(otl_mqtt_format_topic(s_mqtt.touch_event_log_discovery_topic,
                                              sizeof(s_mqtt.touch_event_log_discovery_topic),
                                              "%s/switch/%s_touch_event_logs/config",
                                              CONFIG_OTL_MQTT_DISCOVERY_PREFIX,
                                              s_mqtt.device_id),
                        TAG,
                        "MQTT touch event log discovery topic too long");
    ESP_RETURN_ON_ERROR(otl_mqtt_format_topic(s_mqtt.touch_cal_log_state_topic,
                                              sizeof(s_mqtt.touch_cal_log_state_topic),
                                              "%s/logging/touch_calibration/state",
                                              s_mqtt.topic_root),
                        TAG,
                        "MQTT touch calibration log state topic too long");
    ESP_RETURN_ON_ERROR(otl_mqtt_format_topic(s_mqtt.touch_cal_log_command_topic,
                                              sizeof(s_mqtt.touch_cal_log_command_topic),
                                              "%s/logging/touch_calibration/set",
                                              s_mqtt.topic_root),
                        TAG,
                        "MQTT touch calibration log command topic too long");
    ESP_RETURN_ON_ERROR(otl_mqtt_format_topic(s_mqtt.touch_cal_log_discovery_topic,
                                              sizeof(s_mqtt.touch_cal_log_discovery_topic),
                                              "%s/switch/%s_touch_calibration_logs/config",
                                              CONFIG_OTL_MQTT_DISCOVERY_PREFIX,
                                              s_mqtt.device_id),
                        TAG,
                        "MQTT touch calibration log discovery topic too long");
    ESP_RETURN_ON_ERROR(otl_mqtt_format_topic(s_mqtt.touch_raw_log_state_topic,
                                              sizeof(s_mqtt.touch_raw_log_state_topic),
                                              "%s/logging/touch_raw/state",
                                              s_mqtt.topic_root),
                        TAG,
                        "MQTT touch raw log state topic too long");
    ESP_RETURN_ON_ERROR(otl_mqtt_format_topic(s_mqtt.touch_raw_log_command_topic,
                                              sizeof(s_mqtt.touch_raw_log_command_topic),
                                              "%s/logging/touch_raw/set",
                                              s_mqtt.topic_root),
                        TAG,
                        "MQTT touch raw log command topic too long");
    ESP_RETURN_ON_ERROR(otl_mqtt_format_topic(s_mqtt.touch_raw_log_discovery_topic,
                                              sizeof(s_mqtt.touch_raw_log_discovery_topic),
                                              "%s/switch/%s_touch_raw_logs/config",
                                              CONFIG_OTL_MQTT_DISCOVERY_PREFIX,
                                              s_mqtt.device_id),
                        TAG,
                        "MQTT touch raw log discovery topic too long");
    ESP_RETURN_ON_ERROR(otl_mqtt_format_topic(s_mqtt.pwm_log_state_topic,
                                              sizeof(s_mqtt.pwm_log_state_topic),
                                              "%s/logging/pwm_duty/state",
                                              s_mqtt.topic_root),
                        TAG,
                        "MQTT PWM log state topic too long");
    ESP_RETURN_ON_ERROR(otl_mqtt_format_topic(s_mqtt.pwm_log_command_topic,
                                              sizeof(s_mqtt.pwm_log_command_topic),
                                              "%s/logging/pwm_duty/set",
                                              s_mqtt.topic_root),
                        TAG,
                        "MQTT PWM log command topic too long");
    ESP_RETURN_ON_ERROR(otl_mqtt_format_topic(s_mqtt.pwm_log_discovery_topic,
                                              sizeof(s_mqtt.pwm_log_discovery_topic),
                                              "%s/switch/%s_pwm_duty_logs/config",
                                              CONFIG_OTL_MQTT_DISCOVERY_PREFIX,
                                              s_mqtt.device_id),
                        TAG,
                        "MQTT PWM log discovery topic too long");
#if CONFIG_OTL_PRESENCE_SENSOR
    ESP_RETURN_ON_ERROR(otl_mqtt_format_topic(s_mqtt.radar_log_state_topic,
                                              sizeof(s_mqtt.radar_log_state_topic),
                                              "%s/logging/radar_status/state",
                                              s_mqtt.topic_root),
                        TAG,
                        "MQTT radar log state topic too long");
    ESP_RETURN_ON_ERROR(otl_mqtt_format_topic(s_mqtt.radar_log_command_topic,
                                              sizeof(s_mqtt.radar_log_command_topic),
                                              "%s/logging/radar_status/set",
                                              s_mqtt.topic_root),
                        TAG,
                        "MQTT radar log command topic too long");
    ESP_RETURN_ON_ERROR(otl_mqtt_format_topic(s_mqtt.radar_log_discovery_topic,
                                              sizeof(s_mqtt.radar_log_discovery_topic),
                                              "%s/switch/%s_radar_status_logs/config",
                                              CONFIG_OTL_MQTT_DISCOVERY_PREFIX,
                                              s_mqtt.device_id),
                        TAG,
                        "MQTT radar log discovery topic too long");
    ESP_RETURN_ON_ERROR(otl_mqtt_format_topic(s_mqtt.occupancy_auto_off_state_topic,
                                              sizeof(s_mqtt.occupancy_auto_off_state_topic),
                                              "%s/occupancy/auto_off/state",
                                              s_mqtt.topic_root),
                        TAG,
                        "MQTT occupancy auto-off state topic too long");
    ESP_RETURN_ON_ERROR(otl_mqtt_format_topic(s_mqtt.occupancy_auto_off_command_topic,
                                              sizeof(s_mqtt.occupancy_auto_off_command_topic),
                                              "%s/occupancy/auto_off/set",
                                              s_mqtt.topic_root),
                        TAG,
                        "MQTT occupancy auto-off command topic too long");
    ESP_RETURN_ON_ERROR(otl_mqtt_format_topic(s_mqtt.occupancy_auto_off_discovery_topic,
                                              sizeof(s_mqtt.occupancy_auto_off_discovery_topic),
                                              "%s/switch/%s_occupancy_auto_off/config",
                                              CONFIG_OTL_MQTT_DISCOVERY_PREFIX,
                                              s_mqtt.device_id),
                        TAG,
                        "MQTT occupancy auto-off discovery topic too long");
#endif
    ESP_RETURN_ON_ERROR(otl_mqtt_format_topic(s_mqtt.firmware_version_state_topic,
                                              sizeof(s_mqtt.firmware_version_state_topic),
                                              "%s/firmware/version/state",
                                              s_mqtt.topic_root),
                        TAG,
                        "MQTT firmware version state topic too long");
    ESP_RETURN_ON_ERROR(otl_mqtt_format_topic(s_mqtt.firmware_version_discovery_topic,
                                              sizeof(s_mqtt.firmware_version_discovery_topic),
                                              "%s/sensor/%s_firmware_version/config",
                                              CONFIG_OTL_MQTT_DISCOVERY_PREFIX,
                                              s_mqtt.device_id),
                        TAG,
                        "MQTT firmware version discovery topic too long");
    ESP_RETURN_ON_ERROR(otl_mqtt_format_topic(s_mqtt.firmware_built_state_topic,
                                              sizeof(s_mqtt.firmware_built_state_topic),
                                              "%s/firmware/built/state",
                                              s_mqtt.topic_root),
                        TAG,
                        "MQTT firmware built state topic too long");
    ESP_RETURN_ON_ERROR(otl_mqtt_format_topic(s_mqtt.firmware_built_discovery_topic,
                                              sizeof(s_mqtt.firmware_built_discovery_topic),
                                              "%s/sensor/%s_firmware_built/config",
                                              CONFIG_OTL_MQTT_DISCOVERY_PREFIX,
                                              s_mqtt.device_id),
                        TAG,
                        "MQTT firmware built discovery topic too long");
    ESP_RETURN_ON_ERROR(otl_mqtt_format_topic(s_mqtt.firmware_idf_state_topic,
                                              sizeof(s_mqtt.firmware_idf_state_topic),
                                              "%s/firmware/idf/state",
                                              s_mqtt.topic_root),
                        TAG,
                        "MQTT firmware ESP-IDF state topic too long");
    ESP_RETURN_ON_ERROR(otl_mqtt_format_topic(s_mqtt.firmware_idf_discovery_topic,
                                              sizeof(s_mqtt.firmware_idf_discovery_topic),
                                              "%s/sensor/%s_firmware_idf/config",
                                              CONFIG_OTL_MQTT_DISCOVERY_PREFIX,
                                              s_mqtt.device_id),
                        TAG,
                        "MQTT firmware ESP-IDF discovery topic too long");
    ESP_RETURN_ON_ERROR(otl_mqtt_format_topic(s_mqtt.firmware_target_state_topic,
                                              sizeof(s_mqtt.firmware_target_state_topic),
                                              "%s/firmware/target/state",
                                              s_mqtt.topic_root),
                        TAG,
                        "MQTT firmware target state topic too long");
    ESP_RETURN_ON_ERROR(otl_mqtt_format_topic(s_mqtt.firmware_target_discovery_topic,
                                              sizeof(s_mqtt.firmware_target_discovery_topic),
                                              "%s/sensor/%s_firmware_target/config",
                                              CONFIG_OTL_MQTT_DISCOVERY_PREFIX,
                                              s_mqtt.device_id),
                        TAG,
                        "MQTT firmware target discovery topic too long");
    ESP_RETURN_ON_ERROR(otl_mqtt_format_topic(s_mqtt.firmware_homekit_state_topic,
                                              sizeof(s_mqtt.firmware_homekit_state_topic),
                                              "%s/firmware/homekit/state",
                                              s_mqtt.topic_root),
                        TAG,
                        "MQTT firmware HomeKit state topic too long");
    ESP_RETURN_ON_ERROR(otl_mqtt_format_topic(s_mqtt.firmware_homekit_discovery_topic,
                                              sizeof(s_mqtt.firmware_homekit_discovery_topic),
                                              "%s/binary_sensor/%s_firmware_homekit/config",
                                              CONFIG_OTL_MQTT_DISCOVERY_PREFIX,
                                              s_mqtt.device_id),
                        TAG,
                        "MQTT firmware HomeKit discovery topic too long");
    ESP_RETURN_ON_ERROR(otl_mqtt_format_topic(s_mqtt.firmware_presence_state_topic,
                                              sizeof(s_mqtt.firmware_presence_state_topic),
                                              "%s/firmware/presence_sensor/state",
                                              s_mqtt.topic_root),
                        TAG,
                        "MQTT firmware presence sensor state topic too long");
    ESP_RETURN_ON_ERROR(otl_mqtt_format_topic(s_mqtt.firmware_presence_discovery_topic,
                                              sizeof(s_mqtt.firmware_presence_discovery_topic),
                                              "%s/binary_sensor/%s_firmware_presence_sensor/config",
                                              CONFIG_OTL_MQTT_DISCOVERY_PREFIX,
                                              s_mqtt.device_id),
                        TAG,
                        "MQTT firmware presence sensor discovery topic too long");
    ESP_RETURN_ON_ERROR(otl_mqtt_format_topic(s_mqtt.firmware_circadian_state_topic,
                                              sizeof(s_mqtt.firmware_circadian_state_topic),
                                              "%s/firmware/circadian/state",
                                              s_mqtt.topic_root),
                        TAG,
                        "MQTT firmware circadian state topic too long");
    ESP_RETURN_ON_ERROR(otl_mqtt_format_topic(s_mqtt.firmware_circadian_discovery_topic,
                                              sizeof(s_mqtt.firmware_circadian_discovery_topic),
                                              "%s/binary_sensor/%s_firmware_circadian/config",
                                              CONFIG_OTL_MQTT_DISCOVERY_PREFIX,
                                              s_mqtt.device_id),
                        TAG,
                        "MQTT firmware circadian discovery topic too long");
    ESP_RETURN_ON_ERROR(otl_mqtt_format_topic(s_mqtt.last_event_state_topic,
                                              sizeof(s_mqtt.last_event_state_topic),
                                              "%s/events/last/state",
                                              s_mqtt.topic_root),
                        TAG,
                        "MQTT last event state topic too long");
    ESP_RETURN_ON_ERROR(otl_mqtt_format_topic(s_mqtt.last_event_discovery_topic,
                                              sizeof(s_mqtt.last_event_discovery_topic),
                                              "%s/sensor/%s_last_event/config",
                                              CONFIG_OTL_MQTT_DISCOVERY_PREFIX,
                                              s_mqtt.device_id),
                        TAG,
                        "MQTT last event discovery topic too long");

    return ESP_OK;
}

static void otl_mqtt_event_handler(void *handler_args,
                                   esp_event_base_t base,
                                   int32_t event_id,
                                   void *event_data)
{
    (void)handler_args;
    (void)base;

    esp_mqtt_event_handle_t event = event_data;

    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED:
            s_mqtt.connected = true;
            ESP_LOGI(TAG, "Connected to broker");
            esp_mqtt_client_subscribe(s_mqtt.client, s_mqtt.light_command_topic, 1);
            esp_mqtt_client_subscribe(s_mqtt.client, s_mqtt.brightness_command_topic, 1);
            esp_mqtt_client_subscribe(s_mqtt.client, s_mqtt.color_temp_command_topic, 1);
#if CONFIG_OTL_CIRCADIAN_ENABLE
            esp_mqtt_client_subscribe(s_mqtt.client, s_mqtt.circadian_enabled_command_topic, 1);
            esp_mqtt_client_subscribe(s_mqtt.client, s_mqtt.circadian_coolest_command_topic, 1);
            esp_mqtt_client_subscribe(s_mqtt.client, s_mqtt.circadian_warmest_command_topic, 1);
            esp_mqtt_client_subscribe(s_mqtt.client, s_mqtt.circadian_morning_ramp_command_topic, 1);
#endif
            esp_mqtt_client_subscribe(s_mqtt.client, s_mqtt.led_thermal_limit_command_topic, 1);
            esp_mqtt_client_subscribe(s_mqtt.client, s_mqtt.verbose_diag_command_topic, 1);
            esp_mqtt_client_subscribe(s_mqtt.client, s_mqtt.status_log_command_topic, 1);
            esp_mqtt_client_subscribe(s_mqtt.client, s_mqtt.sensor_debug_command_topic, 1);
            esp_mqtt_client_subscribe(s_mqtt.client, s_mqtt.touch_event_log_command_topic, 1);
            esp_mqtt_client_subscribe(s_mqtt.client, s_mqtt.touch_cal_log_command_topic, 1);
            esp_mqtt_client_subscribe(s_mqtt.client, s_mqtt.touch_raw_log_command_topic, 1);
            esp_mqtt_client_subscribe(s_mqtt.client, s_mqtt.pwm_log_command_topic, 1);
#if CONFIG_OTL_PRESENCE_SENSOR
            esp_mqtt_client_subscribe(s_mqtt.client, s_mqtt.radar_log_command_topic, 1);
            esp_mqtt_client_subscribe(s_mqtt.client, s_mqtt.radar_motion_max_distance_command_topic, 1);
            esp_mqtt_client_subscribe(s_mqtt.client, s_mqtt.radar_stationary_max_distance_command_topic, 1);
            esp_mqtt_client_subscribe(s_mqtt.client, s_mqtt.occupancy_auto_off_command_topic, 1);
#endif
            otl_mqtt_cleanup_stale_discovery();
            otl_mqtt_publish_discovery();
            otl_mqtt_publish(s_mqtt.availability_topic, "online", true);
            otl_mqtt_publish_build_info();
            otl_state_notify_current(OTL_CHANGE_SOURCE_SYSTEM);
            otl_telemetry_notify_current();
            otl_runtime_settings_notify_current(OTL_CHANGE_SOURCE_SYSTEM);
            otl_event_notify_current();
            otl_event_emit(OTL_EVENT_LEVEL_INFO, "mqtt", "MQTT connected");
            break;
        case MQTT_EVENT_DISCONNECTED:
            s_mqtt.connected = false;
            ESP_LOGW(TAG, "Disconnected from broker");
            otl_event_emit(OTL_EVENT_LEVEL_WARNING, "mqtt", "MQTT disconnected");
            break;
        case MQTT_EVENT_DATA:
            otl_mqtt_handle_command(event);
            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGW(TAG, "MQTT client reported an error");
            break;
        default:
            break;
    }
}

static esp_err_t otl_mqtt_init_client(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {0};

    if (s_mqtt.started) {
        return ESP_OK;
    }

    ESP_RETURN_ON_ERROR(otl_mqtt_build_topics(), TAG, "Failed to build MQTT topics");

    mqtt_cfg.broker.address.uri = CONFIG_OTL_MQTT_BROKER_URI;
    mqtt_cfg.credentials.username = (strlen(CONFIG_OTL_MQTT_USERNAME) > 0) ? CONFIG_OTL_MQTT_USERNAME : NULL;
    mqtt_cfg.credentials.authentication.password =
        (strlen(CONFIG_OTL_MQTT_PASSWORD) > 0) ? CONFIG_OTL_MQTT_PASSWORD : NULL;
    mqtt_cfg.session.last_will.topic = s_mqtt.availability_topic;
    mqtt_cfg.session.last_will.msg = "offline";
    mqtt_cfg.session.last_will.qos = 1;
    mqtt_cfg.session.last_will.retain = true;

    s_mqtt.client = esp_mqtt_client_init(&mqtt_cfg);
    if (s_mqtt.client == NULL) {
        return ESP_FAIL;
    }

    ESP_RETURN_ON_ERROR(esp_mqtt_client_register_event(s_mqtt.client,
                                                       ESP_EVENT_ANY_ID,
                                                       otl_mqtt_event_handler,
                                                       NULL),
                        TAG,
                        "Failed to register MQTT events");
    ESP_RETURN_ON_ERROR(esp_mqtt_client_start(s_mqtt.client), TAG, "Failed to start MQTT client");

    s_mqtt.started = true;
    ESP_LOGI(TAG, "MQTT integration started");
    return ESP_OK;
}

static void otl_mqtt_task(void *arg)
{
    (void)arg;

    ESP_LOGI(TAG, "Waiting for WiFi before starting MQTT");
    (void)otl_net_wait_for_wifi(portMAX_DELAY);

    if (!otl_net_wifi_is_connected()) {
        ESP_LOGE(TAG, "WiFi wait returned without an active connection");
        vTaskDelete(NULL);
        return;
    }

    if (otl_mqtt_init_client() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize MQTT client");
    }

    vTaskDelete(NULL);
}

esp_err_t otl_mqtt_start(void)
{
    if (s_mqtt.start_requested) {
        return ESP_OK;
    }

    if (!s_mqtt.state_listener_registered) {
        ESP_RETURN_ON_ERROR(otl_state_add_listener(otl_mqtt_state_listener, NULL),
                            TAG,
                            "Failed to register state listener");
        s_mqtt.state_listener_registered = true;
    }
    if (!s_mqtt.telemetry_listener_registered) {
        ESP_RETURN_ON_ERROR(otl_telemetry_add_listener(otl_mqtt_telemetry_listener, NULL),
                            TAG,
                            "Failed to register telemetry listener");
        s_mqtt.telemetry_listener_registered = true;
    }
    if (!s_mqtt.settings_listener_registered) {
        ESP_RETURN_ON_ERROR(otl_runtime_settings_add_listener(otl_mqtt_settings_listener, NULL),
                            TAG,
                            "Failed to register settings listener");
        s_mqtt.settings_listener_registered = true;
    }
    if (!s_mqtt.event_listener_registered) {
        ESP_RETURN_ON_ERROR(otl_event_add_listener(otl_mqtt_event_listener, NULL),
                            TAG,
                            "Failed to register event listener");
        s_mqtt.event_listener_registered = true;
    }

    if (xTaskCreate(otl_mqtt_task,
                    OTL_MQTT_TASK_NAME,
                    OTL_MQTT_TASK_STACK_SIZE,
                    NULL,
                    OTL_MQTT_TASK_PRIORITY,
                    NULL) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }

    s_mqtt.start_requested = true;
    return ESP_OK;
}

#else

esp_err_t otl_mqtt_start(void)
{
    return ESP_OK;
}

#endif
