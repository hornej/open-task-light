#include "otl_circadian.h"

#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>

#include "sdkconfig.h"

#include "esp_log.h"
#include "esp_sntp.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "otl_net.h"

#if CONFIG_OTL_CIRCADIAN_ENABLE

static const char *TAG = "otl_circadian";

static bool s_started;

static otl_circadian_apply_fn_t s_apply_cb;
static void *s_apply_ctx;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
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

static void log_time_sync_success(void)
{
    time_t now = 0;
    struct tm timeinfo = {0};
    char ts[32] = {0};

    time(&now);
    localtime_r(&now, &timeinfo);
    if (strftime(ts, sizeof(ts), "%Y-%m-%d %H:%M:%S", &timeinfo) == 0) {
        ESP_LOGI(TAG, "SNTP time sync complete");
        return;
    }

    ESP_LOGI(TAG,
             "SNTP time sync complete: %s (%s via %s)",
             ts,
             CONFIG_OTL_TIMEZONE,
             CONFIG_OTL_SNTP_SERVER);
}

#define SECONDS_PER_DAY 86400

static bool parse_hhmm_time(const char *value, int *seconds_out)
{
    if (value == NULL || seconds_out == NULL) {
        return false;
    }

    const char *colon = strchr(value, ':');
    if (colon == NULL) {
        return false;
    }
    if (strchr(colon + 1, ':') != NULL) {
        return false;
    }

    char *endptr = NULL;
    long hour = strtol(value, &endptr, 10);
    if (endptr != colon) {
        return false;
    }

    long minute = strtol(colon + 1, &endptr, 10);
    if (endptr == (colon + 1) || *endptr != '\0') {
        return false;
    }

    if (hour < 0 || hour > 23 || minute < 0 || minute > 59) {
        return false;
    }

    *seconds_out = (int)(hour * 3600L + minute * 60L);
    return true;
}

static int parse_config_time_or_fallback(const char *config_value,
                                         int fallback_seconds,
                                         const char *config_name)
{
    int parsed_seconds = 0;
    if (parse_hhmm_time(config_value, &parsed_seconds)) {
        return parsed_seconds;
    }

    int fb_hour = fallback_seconds / 3600;
    int fb_minute = (fallback_seconds % 3600) / 60;
    ESP_LOGW(TAG,
             "%s has invalid HH:MM value '%s'; using fallback %02d:%02d",
             config_name,
             (config_value != NULL) ? config_value : "",
             fb_hour,
             fb_minute);
    return fallback_seconds;
}

static float circadian_compute_cool_ratio(const struct tm *local_time,
                                          int coolest_sec,
                                          int warmest_sec)
{
    const int sec_of_day = (local_time->tm_hour * 3600) + (local_time->tm_min * 60) + local_time->tm_sec;

    const float min_ratio = clampf((float)CONFIG_OTL_CIRCADIAN_COOL_MIN_PCT / 100.0f, 0.0f, 1.0f);
    const float max_ratio = clampf((float)CONFIG_OTL_CIRCADIAN_COOL_MAX_PCT / 100.0f, 0.0f, 1.0f);
    const float lo = (min_ratio <= max_ratio) ? min_ratio : max_ratio;
    const float hi = (min_ratio <= max_ratio) ? max_ratio : min_ratio;

    float base = 0.0f;
    if (coolest_sec == warmest_sec) {
        // Fallback for invalid configuration: keep a 24h sinusoid with coolest
        // at the configured peak and warmest 12h later.
        const float seconds_delta = (float)(sec_of_day - coolest_sec);
        const float phase = (2.0f * (float)M_PI * seconds_delta) / (float)SECONDS_PER_DAY;
        base = 0.5f + 0.5f * cosf(phase);  // 1 @ coolest, 0 @ coolest+12h
    } else {
        // Piecewise cosine interpolation between warmest and coolest times.
        int rise_duration_sec = coolest_sec - warmest_sec;
        if (rise_duration_sec < 0) {
            rise_duration_sec += SECONDS_PER_DAY;
        }

        int elapsed_from_warmest = sec_of_day - warmest_sec;
        if (elapsed_from_warmest < 0) {
            elapsed_from_warmest += SECONDS_PER_DAY;
        }

        if (elapsed_from_warmest <= rise_duration_sec) {
            float t = (float)elapsed_from_warmest / (float)rise_duration_sec; // 0..1
            base = 0.5f - 0.5f * cosf((float)M_PI * t);                        // 0->1
        } else {
            const int fall_duration_sec = SECONDS_PER_DAY - rise_duration_sec;
            const int elapsed_fall = elapsed_from_warmest - rise_duration_sec;
            float t = (float)elapsed_fall / (float)fall_duration_sec;          // 0..1
            base = 0.5f + 0.5f * cosf((float)M_PI * t);                        // 1->0
        }
    }

    return clampf(lo + (hi - lo) * base, 0.0f, 1.0f);
}

static void circadian_task(void *arg)
{
    (void)arg;

    esp_err_t err = otl_net_start_wifi();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "WiFi start failed: %s", esp_err_to_name(err));
        vTaskDelete(NULL);
        return;
    }

    while (1) {
        err = otl_net_wait_for_wifi(pdMS_TO_TICKS(30000));
        if (err == ESP_OK) {
            break;
        }

        ESP_LOGW(TAG, "WiFi connect timed out; retrying");
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
    log_time_sync_success();

    const int coolest_sec = parse_config_time_or_fallback(
        CONFIG_OTL_CIRCADIAN_COOLEST_TIME,
        11 * 3600,
        "CONFIG_OTL_CIRCADIAN_COOLEST_TIME");
    const int warmest_sec = parse_config_time_or_fallback(
        CONFIG_OTL_CIRCADIAN_WARMEST_TIME,
        23 * 3600,
        "CONFIG_OTL_CIRCADIAN_WARMEST_TIME");

    float last_ratio = -1.0f;
    while (1) {
        time_t now = 0;
        struct tm timeinfo = {0};
        time(&now);
        localtime_r(&now, &timeinfo);

        if (timeinfo.tm_year >= (2020 - 1900)) {
            float ratio = circadian_compute_cool_ratio(&timeinfo, coolest_sec, warmest_sec);
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
