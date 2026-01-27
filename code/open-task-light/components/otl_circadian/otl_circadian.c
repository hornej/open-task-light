#include "otl_circadian.h"

#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>

#include "sdkconfig.h"

#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_sntp.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "nvs_flash.h"

#if CONFIG_OTL_CIRCADIAN_ENABLE

static const char *TAG = "otl_circadian";

static EventGroupHandle_t s_wifi_event_group;
static int s_wifi_retry_count;
static bool s_started;

static otl_circadian_apply_fn_t s_apply_cb;
static void *s_apply_ctx;

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    (void)arg;
    (void)event_data;

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        (void)esp_wifi_connect();
        return;
    }

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_wifi_retry_count < 10) {
            s_wifi_retry_count++;
            ESP_LOGW(TAG, "WiFi disconnected; retry %d/10", s_wifi_retry_count);
            (void)esp_wifi_connect();
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        return;
    }

    if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        s_wifi_retry_count = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        xEventGroupClearBits(s_wifi_event_group, WIFI_FAIL_BIT);
        return;
    }
}

static esp_err_t wifi_init_sta(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_flash_init failed: %s", esp_err_to_name(err));
        return err;
    }

    err = esp_netif_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }

    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }

    if (esp_netif_create_default_wifi_sta() == NULL) {
        return ESP_FAIL;
    }

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    err = esp_wifi_init(&cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_init failed: %s", esp_err_to_name(err));
        return err;
    }

    if (s_wifi_event_group == NULL) {
        s_wifi_event_group = xEventGroupCreate();
        if (s_wifi_event_group == NULL) {
            return ESP_ERR_NO_MEM;
        }
    }

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));

    wifi_config_t wifi_config = {0};
    strncpy((char *)wifi_config.sta.ssid, CONFIG_OTL_WIFI_SSID, sizeof(wifi_config.sta.ssid) - 1);
    strncpy((char *)wifi_config.sta.password, CONFIG_OTL_WIFI_PASSWORD, sizeof(wifi_config.sta.password) - 1);
    wifi_config.sta.threshold.authmode = WIFI_AUTH_OPEN;
    wifi_config.sta.pmf_cfg.capable = true;
    wifi_config.sta.pmf_cfg.required = false;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    return ESP_OK;
}

static bool time_is_valid(void)
{
    time_t now = 0;
    struct tm timeinfo = {0};
    time(&now);
    localtime_r(&now, &timeinfo);
    // Year since 1900. Anything pre-2020 is almost certainly "not synced".
    return timeinfo.tm_year >= (2020 - 1900);
}

static float circadian_compute_cool_ratio(const struct tm *local_time)
{
    const int sec_of_day = (local_time->tm_hour * 3600) + (local_time->tm_min * 60) + local_time->tm_sec;
    const int peak_sec = (CONFIG_OTL_CIRCADIAN_PEAK_HOUR * 3600) + (CONFIG_OTL_CIRCADIAN_PEAK_MINUTE * 60);

    const float min_ratio = clampf((float)CONFIG_OTL_CIRCADIAN_COOL_MIN_PCT / 100.0f, 0.0f, 1.0f);
    const float max_ratio = clampf((float)CONFIG_OTL_CIRCADIAN_COOL_MAX_PCT / 100.0f, 0.0f, 1.0f);
    const float lo = (min_ratio <= max_ratio) ? min_ratio : max_ratio;
    const float hi = (min_ratio <= max_ratio) ? max_ratio : min_ratio;

    const float seconds_delta = (float)(sec_of_day - peak_sec);
    const float phase = (2.0f * (float)M_PI * seconds_delta) / 86400.0f;
    const float base = 0.5f + 0.5f * cosf(phase);  // 1 @ peak, 0 @ peak+12h

    return clampf(lo + (hi - lo) * base, 0.0f, 1.0f);
}

static void circadian_task(void *arg)
{
    (void)arg;

    if (strlen(CONFIG_OTL_WIFI_SSID) == 0) {
        ESP_LOGE(TAG, "CONFIG_OTL_WIFI_SSID is empty; circadian disabled");
        vTaskDelete(NULL);
        return;
    }

    ESP_ERROR_CHECK(wifi_init_sta());

    while (1) {
        EventBits_t bits = xEventGroupWaitBits(
            s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            pdMS_TO_TICKS(30000)
        );

        if (bits & WIFI_CONNECTED_BIT) {
            break;
        }

        if (bits & WIFI_FAIL_BIT) {
            ESP_LOGW(TAG, "WiFi connect failed; retrying in 10s");
        } else {
            ESP_LOGW(TAG, "WiFi connect timed out; retrying in 10s");
        }

        s_wifi_retry_count = 0;
        xEventGroupClearBits(s_wifi_event_group, WIFI_FAIL_BIT);
        (void)esp_wifi_connect();
        vTaskDelay(pdMS_TO_TICKS(10000));
    }

    setenv("TZ", CONFIG_OTL_TIMEZONE, 1);
    tzset();

    if (!esp_sntp_enabled()) {
        esp_sntp_setoperatingmode(ESP_SNTP_OPMODE_POLL);
        esp_sntp_setservername(0, CONFIG_OTL_SNTP_SERVER);
        esp_sntp_set_sync_mode(SNTP_SYNC_MODE_SMOOTH);
        esp_sntp_init();
    }

    // Wait for time to be set (SNTP). Continue even if it takes a while.
    while (!time_is_valid()) {
        ESP_LOGI(TAG, "Waiting for SNTP time sync...");
        vTaskDelay(pdMS_TO_TICKS(2000));
    }

    float last_ratio = -1.0f;
    while (1) {
        time_t now = 0;
        struct tm timeinfo = {0};
        time(&now);
        localtime_r(&now, &timeinfo);

        if (timeinfo.tm_year >= (2020 - 1900)) {
            float ratio = circadian_compute_cool_ratio(&timeinfo);
            if (last_ratio < 0.0f || fabsf(ratio - last_ratio) >= 0.0005f) {
                last_ratio = ratio;
                if (s_apply_cb != NULL) {
                    s_apply_cb(ratio, s_apply_ctx);
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS((uint32_t)CONFIG_OTL_CIRCADIAN_UPDATE_INTERVAL_SEC * 1000U));
    }
}

esp_err_t otl_circadian_start(otl_circadian_apply_fn_t apply_cb, void *apply_ctx)
{
    if (apply_cb == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_started) {
        return ESP_ERR_INVALID_STATE;
    }

    s_apply_cb = apply_cb;
    s_apply_ctx = apply_ctx;
    s_started = true;

    BaseType_t ok = xTaskCreate(circadian_task, "otl_circadian", 4096, NULL, 3, NULL);
    if (ok != pdPASS) {
        s_started = false;
        s_apply_cb = NULL;
        s_apply_ctx = NULL;
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

#else

esp_err_t otl_circadian_start(otl_circadian_apply_fn_t apply_cb, void *apply_ctx)
{
    (void)apply_cb;
    (void)apply_ctx;
    return ESP_OK;
}

#endif
