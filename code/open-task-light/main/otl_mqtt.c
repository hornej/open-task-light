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

#include "esp_check.h"
#include "esp_app_desc.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "mqtt_client.h"
#include "otl_runtime.h"

static const char *TAG = "otl_mqtt";

#define OTL_MQTT_TOPIC_MAX             256
#define OTL_MQTT_PAYLOAD_MAX           2048
#define OTL_MQTT_DEVICE_ID_MAX         48
#define OTL_MQTT_UNIQUE_ID_MAX         64
#define OTL_MQTT_MAX_BRIGHTNESS_PCT    95.0f
#define OTL_MQTT_WARM_KELVIN           2700.0f
#define OTL_MQTT_COOL_KELVIN           5000.0f
#define OTL_MQTT_WARM_MIRED            (1000000.0f / OTL_MQTT_WARM_KELVIN)
#define OTL_MQTT_COOL_MIRED            (1000000.0f / OTL_MQTT_COOL_KELVIN)

typedef struct {
    esp_mqtt_client_handle_t client;
    bool started;
    bool connected;
    char device_id[OTL_MQTT_DEVICE_ID_MAX];
    char light_unique_id[OTL_MQTT_UNIQUE_ID_MAX];
#if CONFIG_OTL_PRESENCE_SENSOR
    char occupancy_unique_id[OTL_MQTT_UNIQUE_ID_MAX];
    char occupancy_state_topic[OTL_MQTT_TOPIC_MAX];
    char occupancy_discovery_topic[OTL_MQTT_TOPIC_MAX];
#endif
    char availability_topic[OTL_MQTT_TOPIC_MAX];
    char light_state_topic[OTL_MQTT_TOPIC_MAX];
    char light_command_topic[OTL_MQTT_TOPIC_MAX];
    char brightness_state_topic[OTL_MQTT_TOPIC_MAX];
    char brightness_command_topic[OTL_MQTT_TOPIC_MAX];
    char color_temp_state_topic[OTL_MQTT_TOPIC_MAX];
    char color_temp_command_topic[OTL_MQTT_TOPIC_MAX];
    char light_discovery_topic[OTL_MQTT_TOPIC_MAX];
} otl_mqtt_ctx_t;

static otl_mqtt_ctx_t s_mqtt = {0};

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

static bool otl_mqtt_parse_int_payload(esp_mqtt_event_handle_t event, int *value_out)
{
    char payload[32] = {0};
    char *endptr = NULL;
    long parsed = 0;

    if (value_out == NULL || !otl_mqtt_copy_payload(event, payload, sizeof(payload))) {
        return false;
    }

    parsed = strtol(payload, &endptr, 10);
    if (endptr == payload) {
        return false;
    }

    while (*endptr != '\0') {
        if (!isspace((unsigned char)*endptr)) {
            return false;
        }
        ++endptr;
    }

    *value_out = (int)parsed;
    return true;
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

    buf = malloc((size_t)needed + 1);
    if (buf == NULL) {
        va_end(args);
        return NULL;
    }

    (void)vsnprintf(buf, (size_t)needed + 1, fmt, args);
    va_end(args);
    return buf;
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

static void otl_mqtt_publish(const char *topic, const char *payload, bool retain)
{
    if (!s_mqtt.connected || s_mqtt.client == NULL || topic == NULL || payload == NULL) {
        return;
    }

    if (esp_mqtt_client_publish(s_mqtt.client, topic, payload, 0, 1, retain) < 0) {
        ESP_LOGW(TAG, "Publish failed for topic %s", topic);
    }
}

static void otl_mqtt_publish_state(const otl_public_state_t *state)
{
    char brightness_payload[8] = {0};
    char color_temp_payload[8] = {0};

    if (state == NULL) {
        return;
    }

    snprintf(brightness_payload, sizeof(brightness_payload), "%d",
             otl_mqtt_brightness_to_ha(state->brightness_percent));
    snprintf(color_temp_payload, sizeof(color_temp_payload), "%d",
             otl_mqtt_temp_ratio_to_mired(state->temp_ratio));

    otl_mqtt_publish(s_mqtt.light_state_topic, state->is_on ? "ON" : "OFF", true);
    otl_mqtt_publish(s_mqtt.brightness_state_topic, brightness_payload, true);
    otl_mqtt_publish(s_mqtt.color_temp_state_topic, color_temp_payload, true);
#if CONFIG_OTL_PRESENCE_SENSOR
    otl_mqtt_publish(s_mqtt.occupancy_state_topic, state->presence ? "ON" : "OFF", true);
#endif
}

static void otl_mqtt_publish_discovery(void)
{
    const esp_app_desc_t *app_desc = esp_app_get_description();
    char *light_payload = otl_mqtt_alloc_printf(
        "{"
        "\"name\":\"Task Light\","
        "\"unique_id\":\"%s\","
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
        "\"device\":{"
        "\"identifiers\":[\"%s\"],"
        "\"name\":\"%s\","
        "\"manufacturer\":\"Open Task Light\","
        "\"model\":\"Open Task Light\","
        "\"sw_version\":\"%s\""
        "}"
        "}",
        s_mqtt.light_unique_id,
        s_mqtt.light_command_topic,
        s_mqtt.light_state_topic,
        s_mqtt.brightness_command_topic,
        s_mqtt.brightness_state_topic,
        s_mqtt.color_temp_command_topic,
        s_mqtt.color_temp_state_topic,
        (int)lroundf(OTL_MQTT_COOL_MIRED),
        (int)lroundf(OTL_MQTT_WARM_MIRED),
        s_mqtt.availability_topic,
        s_mqtt.device_id,
        CONFIG_OTL_MQTT_DEVICE_NAME,
        app_desc->version);

    if (light_payload == NULL) {
        ESP_LOGE(TAG, "Failed to allocate MQTT discovery payload");
        return;
    }

    otl_mqtt_publish(s_mqtt.light_discovery_topic, light_payload, true);
    free(light_payload);

#if CONFIG_OTL_PRESENCE_SENSOR
    char *occupancy_payload = otl_mqtt_alloc_printf(
        "{"
        "\"name\":\"Occupancy\","
        "\"unique_id\":\"%s\","
        "\"state_topic\":\"%s\","
        "\"device_class\":\"occupancy\","
        "\"payload_on\":\"ON\","
        "\"payload_off\":\"OFF\","
        "\"availability_topic\":\"%s\","
        "\"payload_available\":\"online\","
        "\"payload_not_available\":\"offline\","
        "\"device\":{"
        "\"identifiers\":[\"%s\"],"
        "\"name\":\"%s\","
        "\"manufacturer\":\"Open Task Light\","
        "\"model\":\"Open Task Light\","
        "\"sw_version\":\"%s\""
        "}"
        "}",
        s_mqtt.occupancy_unique_id,
        s_mqtt.occupancy_state_topic,
        s_mqtt.availability_topic,
        s_mqtt.device_id,
        CONFIG_OTL_MQTT_DEVICE_NAME,
        app_desc->version);

    if (occupancy_payload == NULL) {
        ESP_LOGE(TAG, "Failed to allocate occupancy discovery payload");
        return;
    }

    otl_mqtt_publish(s_mqtt.occupancy_discovery_topic, occupancy_payload, true);
    free(occupancy_payload);
#endif
}

static void otl_mqtt_handle_command(esp_mqtt_event_handle_t event)
{
    if (otl_mqtt_topic_matches(event, s_mqtt.light_command_topic)) {
        char payload[8] = {0};
        if (!otl_mqtt_copy_payload(event, payload, sizeof(payload))) {
            return;
        }

        if (strcasecmp(payload, "ON") == 0) {
            otl_state_apply_light_update(&(otl_light_update_t) {
                .set_power = true,
                .power_on = true,
            }, OTL_CHANGE_SOURCE_MQTT);
        } else if (strcasecmp(payload, "OFF") == 0) {
            otl_state_apply_light_update(&(otl_light_update_t) {
                .set_power = true,
                .power_on = false,
            }, OTL_CHANGE_SOURCE_MQTT);
        }
        return;
    }

    if (otl_mqtt_topic_matches(event, s_mqtt.brightness_command_topic)) {
        int brightness_ha = 0;
        if (!otl_mqtt_parse_int_payload(event, &brightness_ha)) {
            return;
        }

        if (brightness_ha <= 0) {
            otl_state_apply_light_update(&(otl_light_update_t) {
                .set_power = true,
                .power_on = false,
            }, OTL_CHANGE_SOURCE_MQTT);
            return;
        }

        otl_state_apply_light_update(&(otl_light_update_t) {
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

        otl_state_apply_light_update(&(otl_light_update_t) {
            .set_temp_ratio = true,
            .temp_ratio = otl_mqtt_mired_to_temp_ratio(mired),
        }, OTL_CHANGE_SOURCE_MQTT);
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

static esp_err_t otl_mqtt_build_topics(void)
{
    uint8_t mac[6] = {0};
    char topic_root[OTL_MQTT_TOPIC_MAX] = {0};

    ESP_ERROR_CHECK(esp_read_mac(mac, ESP_MAC_WIFI_STA));

    snprintf(s_mqtt.device_id,
             sizeof(s_mqtt.device_id),
             "open_task_light_%02x%02x%02x%02x%02x%02x",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    snprintf(s_mqtt.light_unique_id,
             sizeof(s_mqtt.light_unique_id),
             "%s_light",
             s_mqtt.device_id);
#if CONFIG_OTL_PRESENCE_SENSOR
    snprintf(s_mqtt.occupancy_unique_id,
             sizeof(s_mqtt.occupancy_unique_id),
             "%s_occupancy",
             s_mqtt.device_id);
#endif

    ESP_RETURN_ON_ERROR(otl_mqtt_format_topic(topic_root,
                                              sizeof(topic_root),
                                              "%s/%s",
                                              CONFIG_OTL_MQTT_TOPIC_BASE,
                                              s_mqtt.device_id),
                        TAG,
                        "MQTT topic root too long");

    ESP_RETURN_ON_ERROR(otl_mqtt_format_topic(s_mqtt.availability_topic,
                                              sizeof(s_mqtt.availability_topic),
                                              "%s/availability",
                                              topic_root),
                        TAG,
                        "MQTT availability topic too long");
    ESP_RETURN_ON_ERROR(otl_mqtt_format_topic(s_mqtt.light_state_topic,
                                              sizeof(s_mqtt.light_state_topic),
                                              "%s/light/state",
                                              topic_root),
                        TAG,
                        "MQTT light state topic too long");
    ESP_RETURN_ON_ERROR(otl_mqtt_format_topic(s_mqtt.light_command_topic,
                                              sizeof(s_mqtt.light_command_topic),
                                              "%s/light/set",
                                              topic_root),
                        TAG,
                        "MQTT light command topic too long");
    ESP_RETURN_ON_ERROR(otl_mqtt_format_topic(s_mqtt.brightness_state_topic,
                                              sizeof(s_mqtt.brightness_state_topic),
                                              "%s/light/brightness/state",
                                              topic_root),
                        TAG,
                        "MQTT brightness state topic too long");
    ESP_RETURN_ON_ERROR(otl_mqtt_format_topic(s_mqtt.brightness_command_topic,
                                              sizeof(s_mqtt.brightness_command_topic),
                                              "%s/light/brightness/set",
                                              topic_root),
                        TAG,
                        "MQTT brightness command topic too long");
    ESP_RETURN_ON_ERROR(otl_mqtt_format_topic(s_mqtt.color_temp_state_topic,
                                              sizeof(s_mqtt.color_temp_state_topic),
                                              "%s/light/color_temp/state",
                                              topic_root),
                        TAG,
                        "MQTT color temperature state topic too long");
    ESP_RETURN_ON_ERROR(otl_mqtt_format_topic(s_mqtt.color_temp_command_topic,
                                              sizeof(s_mqtt.color_temp_command_topic),
                                              "%s/light/color_temp/set",
                                              topic_root),
                        TAG,
                        "MQTT color temperature command topic too long");
    ESP_RETURN_ON_ERROR(otl_mqtt_format_topic(s_mqtt.light_discovery_topic,
                                              sizeof(s_mqtt.light_discovery_topic),
                                              "%s/light/%s/config",
                                              CONFIG_OTL_MQTT_DISCOVERY_PREFIX,
                                              s_mqtt.device_id),
                        TAG,
                        "MQTT discovery topic too long");
#if CONFIG_OTL_PRESENCE_SENSOR
    ESP_RETURN_ON_ERROR(otl_mqtt_format_topic(s_mqtt.occupancy_state_topic,
                                              sizeof(s_mqtt.occupancy_state_topic),
                                              "%s/occupancy/state",
                                              topic_root),
                        TAG,
                        "MQTT occupancy state topic too long");
    ESP_RETURN_ON_ERROR(otl_mqtt_format_topic(s_mqtt.occupancy_discovery_topic,
                                              sizeof(s_mqtt.occupancy_discovery_topic),
                                              "%s/binary_sensor/%s_occupancy/config",
                                              CONFIG_OTL_MQTT_DISCOVERY_PREFIX,
                                              s_mqtt.device_id),
                        TAG,
                        "MQTT occupancy discovery topic too long");
#endif

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
            otl_mqtt_publish_discovery();
            otl_mqtt_publish(s_mqtt.availability_topic, "online", true);
            otl_state_notify_current(OTL_CHANGE_SOURCE_SYSTEM);
            break;
        case MQTT_EVENT_DISCONNECTED:
            s_mqtt.connected = false;
            ESP_LOGW(TAG, "Disconnected from broker");
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

esp_err_t otl_mqtt_start(void)
{
    if (s_mqtt.started) {
        return ESP_OK;
    }

    ESP_RETURN_ON_ERROR(otl_mqtt_build_topics(), TAG, "Failed to build MQTT topics");

    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = CONFIG_OTL_MQTT_BROKER_URI,
        .credentials.username = (strlen(CONFIG_OTL_MQTT_USERNAME) > 0) ? CONFIG_OTL_MQTT_USERNAME : NULL,
        .credentials.authentication.password = (strlen(CONFIG_OTL_MQTT_PASSWORD) > 0) ? CONFIG_OTL_MQTT_PASSWORD : NULL,
        .session.last_will.topic = s_mqtt.availability_topic,
        .session.last_will.msg = "offline",
        .session.last_will.qos = 1,
        .session.last_will.retain = true,
    };

    s_mqtt.client = esp_mqtt_client_init(&mqtt_cfg);
    if (s_mqtt.client == NULL) {
        return ESP_FAIL;
    }

    ESP_RETURN_ON_ERROR(otl_state_add_listener(otl_mqtt_state_listener, NULL),
                        TAG,
                        "Failed to register state listener");
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

#else

esp_err_t otl_mqtt_start(void)
{
    return ESP_OK;
}

#endif
