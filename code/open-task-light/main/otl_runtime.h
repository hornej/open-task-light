#pragma once

#include <stdbool.h>
#include <stddef.h>

#include "esp_err.h"

#define OTL_RUNTIME_TIME_STR_LEN        6
#define OTL_RUNTIME_EVENT_TIMESTAMP_LEN 24
#define OTL_RUNTIME_EVENT_CATEGORY_LEN  24
#define OTL_RUNTIME_EVENT_MESSAGE_LEN   160

typedef struct {
    bool is_on;
    float brightness_percent;
    float temp_ratio;
    bool presence;
} otl_public_state_t;

typedef struct {
    bool set_power;
    bool power_on;
    bool set_brightness;
    bool force_on_with_brightness;
    float brightness_percent;
    bool set_temp_ratio;
    float temp_ratio;
} otl_light_update_t;

typedef enum {
    OTL_CHANGE_SOURCE_SYSTEM = 0,
    OTL_CHANGE_SOURCE_TOUCH,
    OTL_CHANGE_SOURCE_OCCUPANCY,
    OTL_CHANGE_SOURCE_CIRCADIAN,
    OTL_CHANGE_SOURCE_MQTT,
    OTL_CHANGE_SOURCE_HOMEKIT,
} otl_change_source_t;

typedef void (*otl_state_listener_fn)(const otl_public_state_t *state,
                                      otl_change_source_t source,
                                      void *ctx);

typedef struct {
    float ambient_lux;
    float ntc_temp_c;
    float chip_temp_c;
    float thermal_brightness_cap_percent;
    bool thermal_limited;
    bool thermal_ntc_hot;
    bool thermal_chip_hot;
    bool wifi_connected;
    int wifi_rssi_dbm;
} otl_telemetry_t;

typedef struct {
    bool circadian_enabled;
    char circadian_coolest_time[OTL_RUNTIME_TIME_STR_LEN];
    char circadian_warmest_time[OTL_RUNTIME_TIME_STR_LEN];
    float led_thermal_limit_c;
    bool verbose_diagnostics_enabled;
} otl_runtime_settings_t;

typedef enum {
    OTL_EVENT_LEVEL_INFO = 0,
    OTL_EVENT_LEVEL_WARNING,
    OTL_EVENT_LEVEL_ERROR,
} otl_event_level_t;

typedef struct {
    char timestamp[OTL_RUNTIME_EVENT_TIMESTAMP_LEN];
    otl_event_level_t level;
    char category[OTL_RUNTIME_EVENT_CATEGORY_LEN];
    char message[OTL_RUNTIME_EVENT_MESSAGE_LEN];
} otl_runtime_event_t;

typedef void (*otl_telemetry_listener_fn)(const otl_telemetry_t *telemetry, void *ctx);
typedef void (*otl_runtime_settings_listener_fn)(const otl_runtime_settings_t *settings,
                                                 otl_change_source_t source,
                                                 void *ctx);
typedef void (*otl_event_listener_fn)(const otl_runtime_event_t *event, void *ctx);

void otl_state_get_public(otl_public_state_t *state);
bool otl_state_apply_light_update(const otl_light_update_t *update,
                                  otl_change_source_t source);
bool otl_state_set_presence(bool presence, otl_change_source_t source);
esp_err_t otl_state_add_listener(otl_state_listener_fn listener, void *ctx);
void otl_state_notify_current(otl_change_source_t source);

void otl_telemetry_get_public(otl_telemetry_t *telemetry);
esp_err_t otl_telemetry_add_listener(otl_telemetry_listener_fn listener, void *ctx);
void otl_telemetry_notify_current(void);

void otl_runtime_settings_get(otl_runtime_settings_t *settings);
esp_err_t otl_runtime_settings_add_listener(otl_runtime_settings_listener_fn listener, void *ctx);
void otl_runtime_settings_notify_current(otl_change_source_t source);
bool otl_runtime_verbose_diagnostics_enabled(void);
bool otl_runtime_circadian_is_enabled(void);
bool otl_runtime_get_circadian_schedule(int *coolest_seconds, int *warmest_seconds);
esp_err_t otl_runtime_set_verbose_diagnostics(bool enabled, otl_change_source_t source);
esp_err_t otl_runtime_set_circadian_enabled(bool enabled, otl_change_source_t source);
esp_err_t otl_runtime_set_circadian_coolest_time(const char *hhmm, otl_change_source_t source);
esp_err_t otl_runtime_set_circadian_warmest_time(const char *hhmm, otl_change_source_t source);
esp_err_t otl_runtime_set_led_thermal_limit_c(float limit_c, otl_change_source_t source);

void otl_event_get_last(otl_runtime_event_t *event);
esp_err_t otl_event_add_listener(otl_event_listener_fn listener, void *ctx);
void otl_event_notify_current(void);
void otl_event_emit(otl_event_level_t level, const char *category, const char *message);
const char *otl_event_level_to_string(otl_event_level_t level);
