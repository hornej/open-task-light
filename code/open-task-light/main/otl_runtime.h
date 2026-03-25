#pragma once

#include <stdbool.h>

#include "esp_err.h"

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

void otl_state_get_public(otl_public_state_t *state);
bool otl_state_apply_light_update(const otl_light_update_t *update,
                                  otl_change_source_t source);
bool otl_state_set_presence(bool presence, otl_change_source_t source);
esp_err_t otl_state_add_listener(otl_state_listener_fn listener, void *ctx);
void otl_state_notify_current(otl_change_source_t source);
