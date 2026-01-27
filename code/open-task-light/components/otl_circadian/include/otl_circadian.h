#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Called whenever the circadian engine computes a new target.
// `cool_ratio` ranges from 0.0 (fully warm) to 1.0 (fully cool).
typedef void (*otl_circadian_apply_fn_t)(float cool_ratio, void *ctx);

// Starts the circadian WiFi/SNTP task(s). Safe to call only once.
//
// Returns:
// - ESP_OK on success
// - ESP_ERR_INVALID_ARG if apply_cb is NULL
// - ESP_ERR_INVALID_STATE if called more than once
esp_err_t otl_circadian_start(otl_circadian_apply_fn_t apply_cb, void *apply_ctx);

#ifdef __cplusplus
}
#endif

