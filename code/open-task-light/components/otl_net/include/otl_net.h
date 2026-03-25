#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Starts the shared Wi-Fi station stack for Open Task Light.
// Safe to call more than once.
esp_err_t otl_net_start_wifi(void);

// Waits for the Wi-Fi station to be connected.
// Returns ESP_OK when connected or ESP_ERR_TIMEOUT on timeout.
esp_err_t otl_net_wait_for_wifi(TickType_t timeout_ticks);

// Returns true when Wi-Fi currently has an IP connection.
bool otl_net_wifi_is_connected(void);

#ifdef __cplusplus
}
#endif
