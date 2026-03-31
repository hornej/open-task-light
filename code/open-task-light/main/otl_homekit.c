#include "otl_homekit.h"

#include "sdkconfig.h"

#if CONFIG_OTL_HOMEKIT_ENABLE

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_app_desc.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hap.h"
#include "hap_apple_chars.h"
#include "hap_apple_servs.h"
#include "otl_net.h"
#include "otl_runtime.h"

static const char *TAG = "otl_homekit";

#define OTL_HOMEKIT_TASK_STACK_SIZE   6144
#define OTL_HOMEKIT_TASK_PRIORITY     4
#define OTL_HOMEKIT_TASK_NAME         "otl_homekit"
#define OTL_HOMEKIT_MAX_BRIGHTNESS_PCT 95.0f
#define OTL_HOMEKIT_WARM_KELVIN       2700.0f
#define OTL_HOMEKIT_COOL_KELVIN       5000.0f
#define OTL_HOMEKIT_WARM_MIRED        (1000000.0f / OTL_HOMEKIT_WARM_KELVIN)
#define OTL_HOMEKIT_COOL_MIRED        (1000000.0f / OTL_HOMEKIT_COOL_KELVIN)

typedef struct {
    bool start_requested;
    bool listener_registered;
    bool ready;
    hap_acc_t *accessory;
    hap_serv_t *light_service;
    hap_char_t *on_char;
    hap_char_t *brightness_char;
    hap_char_t *color_temp_char;
#if CONFIG_OTL_PRESENCE_SENSOR
    hap_serv_t *occupancy_service;
    hap_char_t *occupancy_char;
#endif
} otl_homekit_ctx_t;

static otl_homekit_ctx_t s_homekit = {0};

static float otl_homekit_clampf(float value, float lo, float hi)
{
    if (value < lo) {
        return lo;
    }
    if (value > hi) {
        return hi;
    }
    return value;
}

static int otl_homekit_brightness_to_hap(float brightness_percent)
{
    float ratio = otl_homekit_clampf(brightness_percent, 0.0f, OTL_HOMEKIT_MAX_BRIGHTNESS_PCT);
    ratio /= OTL_HOMEKIT_MAX_BRIGHTNESS_PCT;
    return (int)lroundf(ratio * 100.0f);
}

static float otl_homekit_brightness_from_hap(int brightness_hap)
{
    int clamped = brightness_hap;

    if (clamped < 1) {
        clamped = 1;
    }
    if (clamped > 100) {
        clamped = 100;
    }

    return ((float)clamped / 100.0f) * OTL_HOMEKIT_MAX_BRIGHTNESS_PCT;
}

static uint32_t otl_homekit_temp_ratio_to_mired(float temp_ratio)
{
    float ratio = otl_homekit_clampf(temp_ratio, 0.0f, 1.0f);
    float mired = OTL_HOMEKIT_WARM_MIRED -
                  (ratio * (OTL_HOMEKIT_WARM_MIRED - OTL_HOMEKIT_COOL_MIRED));
    return (uint32_t)lroundf(mired);
}

static float otl_homekit_mired_to_temp_ratio(uint32_t mired)
{
    float clamped = (float)mired;

    if (clamped < OTL_HOMEKIT_COOL_MIRED) {
        clamped = OTL_HOMEKIT_COOL_MIRED;
    }
    if (clamped > OTL_HOMEKIT_WARM_MIRED) {
        clamped = OTL_HOMEKIT_WARM_MIRED;
    }

    return (OTL_HOMEKIT_WARM_MIRED - clamped) /
           (OTL_HOMEKIT_WARM_MIRED - OTL_HOMEKIT_COOL_MIRED);
}

static void otl_homekit_sync_state(const otl_public_state_t *state)
{
    if (!s_homekit.ready || state == NULL) {
        return;
    }

    if (s_homekit.on_char != NULL) {
        hap_val_t val = {
            .b = state->is_on,
        };
        (void)hap_char_update_val(s_homekit.on_char, &val);
    }

    if (s_homekit.brightness_char != NULL) {
        hap_val_t val = {
            .i = otl_homekit_brightness_to_hap(state->brightness_percent),
        };
        (void)hap_char_update_val(s_homekit.brightness_char, &val);
    }

    if (s_homekit.color_temp_char != NULL) {
        hap_val_t val = {
            .u = otl_homekit_temp_ratio_to_mired(state->temp_ratio),
        };
        (void)hap_char_update_val(s_homekit.color_temp_char, &val);
    }

#if CONFIG_OTL_PRESENCE_SENSOR
    if (s_homekit.occupancy_char != NULL) {
        hap_val_t val = {
            .u = state->presence ? 1 : 0,
        };
        (void)hap_char_update_val(s_homekit.occupancy_char, &val);
    }
#endif
}

static void otl_homekit_state_listener(const otl_public_state_t *state,
                                       otl_change_source_t source,
                                       void *ctx)
{
    (void)ctx;

    if (source == OTL_CHANGE_SOURCE_HOMEKIT) {
        return;
    }

    otl_homekit_sync_state(state);
}

static int otl_homekit_identify(hap_acc_t *ha)
{
    (void)ha;
    ESP_LOGI(TAG, "Accessory identified");
    return HAP_SUCCESS;
}

static int otl_homekit_light_write(hap_write_data_t write_data[],
                                   int count,
                                   void *serv_priv,
                                   void *write_priv)
{
    bool explicit_power = false;
    bool explicit_power_on = false;
    bool brightness_zero = false;
    bool has_update = false;
    int ret = HAP_SUCCESS;
    otl_light_update_t update = {0};

    (void)serv_priv;
    (void)write_priv;

    for (int i = 0; i < count; ++i) {
        hap_write_data_t *write = &write_data[i];
        const char *type_uuid = hap_char_get_type_uuid(write->hc);

        *(write->status) = HAP_STATUS_VAL_INVALID;

        if (strcmp(type_uuid, HAP_CHAR_UUID_ON) == 0) {
            explicit_power = true;
            explicit_power_on = write->val.b;
            *(write->status) = HAP_STATUS_SUCCESS;
            continue;
        }

        if (strcmp(type_uuid, HAP_CHAR_UUID_BRIGHTNESS) == 0) {
            if (write->val.i <= 0) {
                brightness_zero = true;
            } else {
                update.set_brightness = true;
                update.brightness_percent = otl_homekit_brightness_from_hap(write->val.i);
                has_update = true;
            }
            *(write->status) = HAP_STATUS_SUCCESS;
            continue;
        }

        if (strcmp(type_uuid, HAP_CHAR_UUID_COLOR_TEMPERATURE) == 0) {
            update.set_temp_ratio = true;
            update.temp_ratio = otl_homekit_mired_to_temp_ratio(write->val.u);
            has_update = true;
            *(write->status) = HAP_STATUS_SUCCESS;
            continue;
        }

        *(write->status) = HAP_STATUS_RES_ABSENT;
        ret = HAP_FAIL;
    }

    if (explicit_power) {
        update.set_power = true;
        update.power_on = explicit_power_on;
        has_update = true;
    } else if (brightness_zero) {
        update.set_power = true;
        update.power_on = false;
        has_update = true;
    }

    if (update.set_brightness && (!explicit_power || explicit_power_on)) {
        update.force_on_with_brightness = true;
    }

    if (has_update) {
        (void)otl_state_apply_light_update(&update, OTL_CHANGE_SOURCE_HOMEKIT);

        otl_public_state_t state = {0};
        otl_state_get_public(&state);
        otl_homekit_sync_state(&state);
    }

    return ret;
}

static void otl_homekit_log_pairing_payload(void)
{
    char *payload = esp_hap_get_setup_payload((char *)CONFIG_OTL_HOMEKIT_SETUP_CODE,
                                              (char *)CONFIG_OTL_HOMEKIT_SETUP_ID,
                                              false,
                                              HAP_CID_LIGHTING);

    ESP_LOGI(TAG, "Pair with Apple Home using code %s", CONFIG_OTL_HOMEKIT_SETUP_CODE);
    if (payload != NULL) {
        ESP_LOGI(TAG, "Setup payload: %s", payload);
        ESP_LOGI(TAG,
                 "QR URL: https://espressif.github.io/esp-homekit-sdk/qrcode.html?data=%s",
                 payload);
        free(payload);
    }
}

static esp_err_t otl_homekit_init_accessory(void)
{
    uint8_t mac[6] = {0};
    const esp_app_desc_t *app_desc = esp_app_get_description();
    otl_public_state_t state = {0};
    hap_acc_t *accessory = NULL;
    hap_serv_t *light_service = NULL;
    hap_char_t *brightness_char = NULL;
    hap_char_t *color_temp_char = NULL;
    char fw_rev[32] = {0};
    char serial_num[24] = {0};
    uint8_t product_data[8] = {'O', 'T', 'L', 'L', 'I', 'G', 'H', 'T'};
    hap_acc_cfg_t cfg = {
        .name = CONFIG_OTL_HOMEKIT_ACCESSORY_NAME,
        .manufacturer = "Basement Labs",
        .model = "Open Task Light",
        .serial_num = serial_num,
        .fw_rev = fw_rev,
        .hw_rev = "1.0",
        .pv = "1.1.0",
        .identify_routine = otl_homekit_identify,
        .cid = HAP_CID_LIGHTING,
    };

    ESP_ERROR_CHECK(esp_read_mac(mac, ESP_MAC_WIFI_STA));
    snprintf(serial_num,
             sizeof(serial_num),
             "%02X%02X%02X%02X%02X%02X",
             mac[0],
             mac[1],
             mac[2],
             mac[3],
             mac[4],
             mac[5]);
    snprintf(fw_rev, sizeof(fw_rev), "%s", app_desc->version);

    otl_state_get_public(&state);

    if (hap_init(HAP_TRANSPORT_WIFI) != HAP_SUCCESS) {
        return ESP_FAIL;
    }

    accessory = hap_acc_create(&cfg);
    if (accessory == NULL) {
        return ESP_FAIL;
    }

    if (hap_acc_add_product_data(accessory, product_data, sizeof(product_data)) != HAP_SUCCESS) {
        return ESP_FAIL;
    }

    light_service = hap_serv_lightbulb_create(state.is_on);
    if (light_service == NULL) {
        return ESP_FAIL;
    }

    if (hap_serv_add_char(light_service,
                          hap_char_name_create((char *)CONFIG_OTL_HOMEKIT_ACCESSORY_NAME)) != HAP_SUCCESS) {
        return ESP_FAIL;
    }

    brightness_char = hap_char_brightness_create(otl_homekit_brightness_to_hap(state.brightness_percent));
    if (brightness_char == NULL || hap_serv_add_char(light_service, brightness_char) != HAP_SUCCESS) {
        return ESP_FAIL;
    }

    color_temp_char = hap_char_color_temperature_create(otl_homekit_temp_ratio_to_mired(state.temp_ratio));
    if (color_temp_char == NULL || hap_serv_add_char(light_service, color_temp_char) != HAP_SUCCESS) {
        return ESP_FAIL;
    }

    hap_serv_set_write_cb(light_service, otl_homekit_light_write);

    if (hap_acc_add_serv(accessory, light_service) != HAP_SUCCESS) {
        return ESP_FAIL;
    }

#if CONFIG_OTL_PRESENCE_SENSOR
    hap_serv_t *occupancy_service = hap_serv_occupancy_sensor_create(state.presence ? 1 : 0);
    if (occupancy_service == NULL) {
        return ESP_FAIL;
    }

    if (hap_serv_add_char(occupancy_service, hap_char_name_create("Occupancy")) != HAP_SUCCESS) {
        return ESP_FAIL;
    }

    if (hap_acc_add_serv(accessory, occupancy_service) != HAP_SUCCESS) {
        return ESP_FAIL;
    }

    s_homekit.occupancy_service = occupancy_service;
    s_homekit.occupancy_char = hap_serv_get_char_by_uuid(occupancy_service,
                                                         HAP_CHAR_UUID_OCCUPANCY_DETECTED);
    if (s_homekit.occupancy_char == NULL) {
        return ESP_FAIL;
    }
#endif

    hap_add_accessory(accessory);

    hap_set_setup_code(CONFIG_OTL_HOMEKIT_SETUP_CODE);
    if (hap_set_setup_id(CONFIG_OTL_HOMEKIT_SETUP_ID) != HAP_SUCCESS) {
        return ESP_FAIL;
    }

    s_homekit.accessory = accessory;
    s_homekit.light_service = light_service;
    s_homekit.on_char = hap_serv_get_char_by_uuid(light_service, HAP_CHAR_UUID_ON);
    s_homekit.brightness_char = brightness_char;
    s_homekit.color_temp_char = color_temp_char;

    if (s_homekit.on_char == NULL || s_homekit.brightness_char == NULL ||
        s_homekit.color_temp_char == NULL) {
        return ESP_FAIL;
    }

    if (hap_start() != HAP_SUCCESS) {
        return ESP_FAIL;
    }

    s_homekit.ready = true;
    otl_state_notify_current(OTL_CHANGE_SOURCE_SYSTEM);
    otl_homekit_log_pairing_payload();
    ESP_LOGI(TAG, "HomeKit integration started");
    otl_event_emit(OTL_EVENT_LEVEL_INFO, "homekit", "HomeKit integration started");
    return ESP_OK;
}

static void otl_homekit_task(void *arg)
{
    (void)arg;

    ESP_LOGI(TAG, "Waiting for WiFi before starting HomeKit");
    (void)otl_net_wait_for_wifi(portMAX_DELAY);

    if (!otl_net_wifi_is_connected()) {
        ESP_LOGE(TAG, "WiFi wait returned without an active connection");
        vTaskDelete(NULL);
        return;
    }

    if (otl_homekit_init_accessory() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize HomeKit");
    }

    vTaskDelete(NULL);
}

esp_err_t otl_homekit_start(void)
{
    if (s_homekit.start_requested) {
        return ESP_OK;
    }

    if (!s_homekit.listener_registered) {
        ESP_RETURN_ON_ERROR(otl_state_add_listener(otl_homekit_state_listener, NULL),
                            TAG,
                            "Failed to register HomeKit state listener");
        s_homekit.listener_registered = true;
    }

    if (xTaskCreate(otl_homekit_task,
                    OTL_HOMEKIT_TASK_NAME,
                    OTL_HOMEKIT_TASK_STACK_SIZE,
                    NULL,
                    OTL_HOMEKIT_TASK_PRIORITY,
                    NULL) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }

    s_homekit.start_requested = true;
    return ESP_OK;
}

#else

esp_err_t otl_homekit_start(void)
{
    return ESP_OK;
}

#endif
