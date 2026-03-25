// otl_main.c - Open Task Light firmware (ESP-IDF, ESP32-S3)
// PWM dimming, capacitive-touch UI, WS2812 RGB status LED, ADC sensors,
// optional LD2410B presence sensor, optional circadian color temperature.

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>     
#include <time.h>
#include "sdkconfig.h"
#include "esp_idf_version.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#if CONFIG_OTL_SERIAL_OUTPUT
#define OTL_LOGI(tag, fmt, ...) ESP_LOGI(tag, fmt, ##__VA_ARGS__)
#define OTL_LOGW(tag, fmt, ...) ESP_LOGW(tag, fmt, ##__VA_ARGS__)
#define OTL_LOGE(tag, fmt, ...) ESP_LOGE(tag, fmt, ##__VA_ARGS__)
#define OTL_LOGD(tag, fmt, ...) ESP_LOGD(tag, fmt, ##__VA_ARGS__)
#define OTL_LOGI_PERIODIC(tag, fmt, ...) do { \
    char _otl_ts[32]; \
    ESP_LOGI(tag, "[%s] " fmt, otl_log_localtime(_otl_ts, sizeof(_otl_ts)), ##__VA_ARGS__); \
} while (0)
#else
#define OTL_LOGI(tag, fmt, ...) do { (void)(tag); (void)(fmt); } while (0)
#define OTL_LOGW(tag, fmt, ...) do { (void)(tag); (void)(fmt); } while (0)
#define OTL_LOGE(tag, fmt, ...) do { (void)(tag); (void)(fmt); } while (0)
#define OTL_LOGD(tag, fmt, ...) do { (void)(tag); (void)(fmt); } while (0)
#define OTL_LOGI_PERIODIC(tag, fmt, ...) do { (void)(tag); (void)(fmt); } while (0)
#endif

#if CONFIG_OTL_SERIAL_OUTPUT && CONFIG_OTL_LOG_STATUS
#define OTL_LOG_STATUSI(fmt, ...) OTL_LOGI_PERIODIC("sensor", fmt, ##__VA_ARGS__)
#else
#define OTL_LOG_STATUSI(fmt, ...) do { (void)(fmt); } while (0)
#endif

#if CONFIG_OTL_SERIAL_OUTPUT && CONFIG_OTL_LOG_TOUCH_EVENTS
#define OTL_LOG_TOUCHI(fmt, ...) OTL_LOGI("touch", fmt, ##__VA_ARGS__)
#else
#define OTL_LOG_TOUCHI(fmt, ...) do { (void)(fmt); } while (0)
#endif

#if CONFIG_OTL_SERIAL_OUTPUT && CONFIG_OTL_LOG_TOUCH_CAL
#define OTL_LOG_TOUCH_CALI(fmt, ...) OTL_LOGI_PERIODIC("touch", fmt, ##__VA_ARGS__)
#else
#define OTL_LOG_TOUCH_CALI(fmt, ...) do { (void)(fmt); } while (0)
#endif

#if CONFIG_OTL_SERIAL_OUTPUT && CONFIG_OTL_LOG_RADAR_STATUS
#define OTL_LOG_RADARI(fmt, ...) OTL_LOGI_PERIODIC("radar", fmt, ##__VA_ARGS__)
#else
#define OTL_LOG_RADARI(fmt, ...) do { (void)(fmt); } while (0)
#endif

#if CONFIG_OTL_SERIAL_OUTPUT
static const char *otl_log_localtime(char *buf, size_t buf_len) __attribute__((unused));
static const char *otl_log_localtime(char *buf, size_t buf_len)
{
    if (buf == NULL || buf_len == 0) {
        return "time=unset";
    }

    time_t now = 0;
    struct tm timeinfo = {0};
    time(&now);
    localtime_r(&now, &timeinfo);

    // Year since 1900. Anything pre-2020 is almost certainly "not synced".
    if (timeinfo.tm_year < (2020 - 1900)) {
        return "time=unset";
    }

    if (strftime(buf, buf_len, "%Y-%m-%d %H:%M:%S", &timeinfo) == 0) {
        return "time=unset";
    }

    return buf;
}
#endif
#include "esp_timer.h"
#include "driver/ledc.h"
#include "driver/touch_pad.h"
#include "esp_adc/adc_oneshot.h"
#include "driver/gpio.h"
#include "driver/temperature_sensor.h"
#if CONFIG_OTL_PRESENCE_SENSOR
#include "driver/uart.h"
#endif
#include "led_strip.h" 
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "freertos/queue.h"
#include "esp_intr_alloc.h"
#include "otl_circadian.h"
#include "otl_net.h"
#include "otl_runtime.h"

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 2, 0)
#define OTL_ADC_ATTEN ADC_ATTEN_DB_12
#else
#define OTL_ADC_ATTEN ADC_ATTEN_DB_11
#endif

static adc_cali_handle_t adc2_cali_handle = NULL;

#if CONFIG_OTL_PRESENCE_SENSOR
// --- Radar UART ---
#define RADAR_UART_NUM UART_NUM_1
static const int RADAR_UART_BAUD = 256000;  // LD2410 default

typedef struct {
    bool    valid;
    uint8_t targetType;    // bit0=moving, bit1=stationary
    uint16_t movingDist;   // cm
    uint8_t movingEnergy;  // 0-100
    uint16_t staticDist;   // cm
    uint8_t staticEnergy;  // 0-100
} radar_sample_t;

static radar_sample_t radar_last_sample = {0};

// Simple UART RX buffer for LD2410 frames
static uint8_t radar_rx_buf[128];
static size_t  radar_rx_len = 0;
    
#endif

#ifndef MIN
#define MIN(a,b) (((a)<(b))?(a):(b))
#endif
#ifndef MAX
#define MAX(a,b) (((a)>(b))?(a):(b))
#endif

// --- Application-wide timing constants ---
#define TOUCH_MEAS_WAIT_MS        250   // Max wait for touch measurement to complete
#define TOUCH_SETTLE_MS           250   // Delay for filter/FSM to settle before calibration
#define SENSOR_READ_INTERVAL_MS   10000 // Interval between sensor log lines
#define THERMAL_POLL_INTERVAL_MS  1000  // Interval between thermal checks
#define RECALIB_INTERVAL_MS       60000 // Interval between touch recalibration attempts
#define RADAR_MAX_ENERGY          100   // Maximum energy value from LD2410 radar
#define TOUCH_STARTUP_MAX_RETRIES 4     // Retry startup calibration if pads are still settling
#define TOUCH_STARTUP_RETRY_MS    150   // Delay between startup calibration retries
#define TOUCH_RECOVERY_DELAY_MS   2000  // Early post-boot recalibration pass
#define TOUCH_VERIFY_DELAY_MS     100   // Delay between touch verification samples
#define TOUCH_VERIFY_SAMPLES      3     // Consecutive samples required for a stuck-touch recovery
#define TOUCH_BAD_PRESSED_PADS    3     // Number of simultaneously "pressed" pads treated as a bad calibration

static inline uint32_t clamp_u32(uint32_t v, uint32_t lo, uint32_t hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

// ---------------------------- Pin definitions ----------------------------
// Touch pads
#define TOUCH_LED_SWITCH   TOUCH_PAD_NUM7       // GPIO7
#define TOUCH_SWITCH_UP    TOUCH_PAD_NUM8       // GPIO8
#define TOUCH_SWITCH_DN    TOUCH_PAD_NUM9       // GPIO9
#define TOUCH_TEMP_UP      TOUCH_PAD_NUM10      // GPIO10
#define TOUCH_TEMP_DN      TOUCH_PAD_NUM11      // GPIO11

#if CONFIG_OTL_PRESENCE_SENSOR
#define RADAR_OUT_GPIO     15   // LD2410B digital OUT (occupancy) - candidate 1
#define RADAR_OUT_GPIO_ALT 14   // LD2410B digital OUT (occupancy) - candidate 2
// Optional: UART pins if you later want full LD2410B frame parsing
// Module pin 39 -> GPIO43, module pin 40 -> GPIO44 on ESP32-S3-MINI-1-N4R2
#define RADAR_TX_GPIO      43   // ESP32 TX (GPIO43) -> radar RX (module pin 39)
#define RADAR_RX_GPIO      44   // ESP32 RX (GPIO44) <- radar TX (module pin 40)
#endif

#define DIM_OUT_COOL_GPIO  36   // PWM output for cool white
#define DIM_OUT_WARM_GPIO  35   // PWM output for warm white
#define WS2812_GPIO        40   // RMT / LED-Strip output pin

#define ALS_GPIO           5    // ALS‑PT19 analog input (ADC)
#define NTC_GPIO           16   // NTC thermistor divider analog input (ADC)

// ---------------------------- LEDC PWM ----------------------------
#define OTL_LEDC_DUTY_RES LEDC_TIMER_12_BIT
#define OTL_LEDC_FREQ_HZ  19531
#define OTL_PWM_MAX_DUTY  ((1U << OTL_LEDC_DUTY_RES) - 1U)

// Minimum PWM "on" pulse width (per channel). Any non-zero duty below this
// threshold is clamped up to avoid sub-min pulses.
//
// 0.3us @ 19.531kHz/12-bit => 24 duty counts.
#define OTL_PWM_MIN_ON_TIME_NS 300ULL // 0.3us
#define OTL_PWM_MIN_ON_DUTY_RAW \
    ((uint32_t)(((OTL_PWM_MIN_ON_TIME_NS * (uint64_t)OTL_LEDC_FREQ_HZ * (uint64_t)OTL_PWM_MAX_DUTY) + 1000000000ULL - 1ULL) / 1000000000ULL))
#define OTL_PWM_MIN_ON_DUTY \
    ((OTL_PWM_MIN_ON_DUTY_RAW < 1U) ? 1U : ((OTL_PWM_MIN_ON_DUTY_RAW > OTL_PWM_MAX_DUTY) ? OTL_PWM_MAX_DUTY : OTL_PWM_MIN_ON_DUTY_RAW))

static const ledc_timer_config_t ledc_timer_cfg = {
    .speed_mode       = LEDC_LOW_SPEED_MODE,
    .duty_resolution  = OTL_LEDC_DUTY_RES,
    .timer_num        = LEDC_TIMER_0,
    // Pick a frequency that keeps the LEDC divider close to 1 for the chosen
    // duty resolution (80MHz / 2^N => ~19.5kHz @ 12-bit, ~9.8kHz @ 13-bit,
    // ~4.9kHz @ 14-bit).
    .freq_hz          = OTL_LEDC_FREQ_HZ,
    .clk_cfg          = LEDC_AUTO_CLK
};

static ledc_channel_config_t ledc_ch_cool = {
    .gpio_num   = DIM_OUT_COOL_GPIO,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel    = LEDC_CHANNEL_0,
    .timer_sel  = LEDC_TIMER_0,
    .duty       = 0,
    .hpoint     = 0
};
static ledc_channel_config_t ledc_ch_warm = {
    .gpio_num   = DIM_OUT_WARM_GPIO,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel    = LEDC_CHANNEL_1,
    .timer_sel  = LEDC_TIMER_0,
    .duty       = 0,
    .hpoint     = 0
};
static const uint32_t PWM_MAX_DUTY = OTL_PWM_MAX_DUTY;
static const uint32_t PWM_MIN_ON_DUTY = OTL_PWM_MIN_ON_DUTY;

// ---------------------------- WS2812 (RMT LED-Strip) ----------------------------
 #define WS2812_NUM_PIXELS  1
 static led_strip_handle_t strip;
 static bool neopixel_enabled = false;

// ---------------------------- PWM fade smoothing -------------------------------
// Fade between duty targets in hardware so we don't "skip" duty steps.
// Used by the hold stepper (0->max in ~1.2s at 12-bit/95% cap).
#define PWM_HOLD_STEP_UPDATE_US       600
#define PWM_FADE_STEP                 1
// Try to keep fade duration roughly constant for small deltas so you don't get
// "step + settle" behavior at low brightness.
#define PWM_FADE_TARGET_TIME_MS       10
// Faster turn-off fade: use a larger duty step so we aren't limited to
// "1 duty count per PWM period" at high brightness.
#define PWM_FADE_OFF_STEP             2
#define PWM_FADE_OFF_TIME_MS          100
#define PWM_FADE_MAX_CYCLES_PER_STEP  1000

static inline uint32_t pwm_clamp_min_on_duty(uint32_t duty)
{
    if (duty == 0) {
        return 0;
    }
    if (duty < PWM_MIN_ON_DUTY) {
        return PWM_MIN_ON_DUTY;
    }
    return duty;
}

#if !CONFIG_OTL_NONOVERLAP_PWM
typedef struct {
    ledc_mode_t    speed_mode;
    ledc_channel_t channel;
} pwm_off_timer_arg_t;

static esp_timer_handle_t pwm_off_timer_cool = NULL;
static esp_timer_handle_t pwm_off_timer_warm = NULL;

static pwm_off_timer_arg_t pwm_off_args_cool = {
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel = LEDC_CHANNEL_0,
};
static pwm_off_timer_arg_t pwm_off_args_warm = {
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel = LEDC_CHANNEL_1,
};

static void pwm_off_timer_cb(void *arg)
{
    const pwm_off_timer_arg_t *a = (const pwm_off_timer_arg_t *)arg;
    (void)ledc_set_duty_and_update(a->speed_mode, a->channel, 0, 0);
}

static inline esp_timer_handle_t pwm_off_timer_for_channel(ledc_channel_t channel)
{
    if (channel == ledc_ch_cool.channel) {
        return pwm_off_timer_cool;
    }
    if (channel == ledc_ch_warm.channel) {
        return pwm_off_timer_warm;
    }
    return NULL;
}

static void pwm_set_duty_smooth_timed(ledc_mode_t speed_mode, ledc_channel_t channel, uint32_t target_duty, uint32_t target_time_ms)
{
    target_duty = MIN(target_duty, PWM_MAX_DUTY);
    if (target_duty != 0) {
        esp_timer_handle_t off_timer = pwm_off_timer_for_channel(channel);
        if (off_timer != NULL) {
            (void)esp_timer_stop(off_timer);
        }
        target_duty = pwm_clamp_min_on_duty(target_duty);
    }

    uint32_t current_duty = ledc_get_duty(speed_mode, channel);
    if (current_duty != 0 && current_duty < PWM_MIN_ON_DUTY) {
        current_duty = PWM_MIN_ON_DUTY;
        ESP_ERROR_CHECK(ledc_set_duty_and_update(speed_mode, channel, current_duty, 0));
    }

    if (target_duty == 0) {
        // Fade down to the minimum on-time duty, then snap to fully off so we
        // never output sub-min pulses.
        if (current_duty == 0) {
            return;
        }
        if (current_duty <= PWM_MIN_ON_DUTY) {
            ESP_ERROR_CHECK(ledc_set_duty_and_update(speed_mode, channel, 0, 0));
            return;
        }

        uint32_t min_duty = PWM_MIN_ON_DUTY;
        uint32_t delta = current_duty - min_duty;

        uint32_t fade_step = PWM_FADE_OFF_STEP;
        if (fade_step == 0) {
            fade_step = 1;
        }
        uint32_t steps_required = (delta + fade_step - 1U) / fade_step;
        if (steps_required < 1U) {
            steps_required = 1U;
        }

        uint32_t total_cycles = (uint32_t)(((uint64_t)ledc_timer_cfg.freq_hz * target_time_ms) / 1000ULL);
        if (total_cycles < steps_required) {
            total_cycles = steps_required;
        }

        uint32_t cycles_per_step = total_cycles / steps_required;
        cycles_per_step = clamp_u32(cycles_per_step, 1, PWM_FADE_MAX_CYCLES_PER_STEP);

        ESP_ERROR_CHECK(ledc_set_fade_with_step(speed_mode, channel, min_duty, fade_step, cycles_per_step));
        ESP_ERROR_CHECK(ledc_fade_start(speed_mode, channel, LEDC_FADE_NO_WAIT));

        uint64_t actual_cycles = (uint64_t)steps_required * (uint64_t)cycles_per_step;
        uint64_t actual_time_us = (actual_cycles * 1000000ULL) / (uint64_t)ledc_timer_cfg.freq_hz;
        if (actual_time_us < 1ULL) {
            actual_time_us = 1ULL;
        }

        esp_timer_handle_t off_timer = pwm_off_timer_for_channel(channel);
        if (off_timer != NULL) {
            (void)esp_timer_stop(off_timer);
            ESP_ERROR_CHECK(esp_timer_start_once(off_timer, actual_time_us));
        }
        return;
    }

    // Prevent fade-up from 0 through sub-min duty values.
    if (current_duty == 0) {
        ESP_ERROR_CHECK(ledc_set_duty_and_update(speed_mode, channel, PWM_MIN_ON_DUTY, 0));
        current_duty = PWM_MIN_ON_DUTY;
        if (current_duty == target_duty) {
            return;
        }
    }

    if (current_duty == target_duty) {
        return;
    }

    uint32_t delta = (current_duty > target_duty) ? (current_duty - target_duty) : (target_duty - current_duty);

    uint32_t total_cycles = (uint32_t)(((uint64_t)ledc_timer_cfg.freq_hz * target_time_ms) / 1000ULL);
    if (total_cycles < delta) {
        total_cycles = delta;
    }

    uint32_t cycles_per_step = total_cycles / delta;
    cycles_per_step = clamp_u32(cycles_per_step, 1, PWM_FADE_MAX_CYCLES_PER_STEP);

    ESP_ERROR_CHECK(ledc_set_fade_with_step(speed_mode, channel, target_duty, PWM_FADE_STEP, cycles_per_step));
    ESP_ERROR_CHECK(ledc_fade_start(speed_mode, channel, LEDC_FADE_NO_WAIT));
}

static void pwm_set_duty_smooth(ledc_mode_t speed_mode, ledc_channel_t channel, uint32_t target_duty)
{
    pwm_set_duty_smooth_timed(speed_mode, channel, target_duty, PWM_FADE_TARGET_TIME_MS);
}

static void pwm_set_warm_cool_overlap(uint32_t warm_width, uint32_t cool_width)
{
    warm_width = MIN(warm_width, PWM_MAX_DUTY);
    cool_width = MIN(cool_width, PWM_MAX_DUTY);
    warm_width = pwm_clamp_min_on_duty(warm_width);
    cool_width = pwm_clamp_min_on_duty(cool_width);

    if (warm_width != 0 || cool_width != 0) {
        if (pwm_off_timer_warm != NULL) (void)esp_timer_stop(pwm_off_timer_warm);
        if (pwm_off_timer_cool != NULL) (void)esp_timer_stop(pwm_off_timer_cool);
    }
    ESP_ERROR_CHECK(ledc_set_duty_and_update(ledc_ch_warm.speed_mode, ledc_ch_warm.channel, warm_width, 0));
    ESP_ERROR_CHECK(ledc_set_duty_and_update(ledc_ch_cool.speed_mode, ledc_ch_cool.channel, cool_width, 0));
}
#endif

#if CONFIG_OTL_NONOVERLAP_PWM
#define PWM_SOFT_FADE_UPDATE_US 1000

typedef struct {
    uint32_t start_warm;
    uint32_t start_cool;
    uint32_t target_warm;
    uint32_t target_cool;
    int64_t  start_us;
    int64_t  duration_us;
    bool     active;
} pwm_nonoverlap_ramp_t;

static SemaphoreHandle_t pwm_nonoverlap_mutex = NULL;
static esp_timer_handle_t pwm_nonoverlap_timer = NULL;
static pwm_nonoverlap_ramp_t pwm_nonoverlap_ramp = {0};
static uint32_t pwm_nonoverlap_current_warm = 0;
static uint32_t pwm_nonoverlap_current_cool = 0;

static void pwm_set_warm_cool_nonoverlap(uint32_t warm_width, uint32_t cool_width)
{
    // LEDC: duty is pulse width, hpoint is start (phase). Keep pulses within
    // the same PWM period so they never overlap.
    warm_width = MIN(warm_width, PWM_MAX_DUTY);
    cool_width = MIN(cool_width, PWM_MAX_DUTY);

    warm_width = pwm_clamp_min_on_duty(warm_width);
    cool_width = pwm_clamp_min_on_duty(cool_width);

    if ((warm_width + cool_width) > PWM_MAX_DUTY) {
        if (warm_width >= PWM_MAX_DUTY) {
            warm_width = PWM_MAX_DUTY;
            cool_width = 0;
        } else if (cool_width >= PWM_MAX_DUTY) {
            cool_width = PWM_MAX_DUTY;
            warm_width = 0;
        } else {
            cool_width = PWM_MAX_DUTY - warm_width;
        }
    }

    // If the remaining budget can't accommodate a sub-min non-zero pulse, drop
    // that channel entirely (otherwise we'd output a <min pulse width).
    if (warm_width != 0 && warm_width < PWM_MIN_ON_DUTY) {
        cool_width = MIN(cool_width + warm_width, PWM_MAX_DUTY);
        warm_width = 0;
        cool_width = pwm_clamp_min_on_duty(cool_width);
    }
    if (cool_width != 0 && cool_width < PWM_MIN_ON_DUTY) {
        warm_width = MIN(warm_width + cool_width, PWM_MAX_DUTY);
        cool_width = 0;
        warm_width = pwm_clamp_min_on_duty(warm_width);
    }

    uint32_t prev_warm_width = pwm_nonoverlap_current_warm;

    // Update ordering matters to avoid transient overlap while changing hpoint.
    if (warm_width > prev_warm_width) {
        ESP_ERROR_CHECK(ledc_set_duty_and_update(ledc_ch_cool.speed_mode, ledc_ch_cool.channel, cool_width, warm_width));
        ESP_ERROR_CHECK(ledc_set_duty_and_update(ledc_ch_warm.speed_mode, ledc_ch_warm.channel, warm_width, 0));
    } else {
        ESP_ERROR_CHECK(ledc_set_duty_and_update(ledc_ch_warm.speed_mode, ledc_ch_warm.channel, warm_width, 0));
        ESP_ERROR_CHECK(ledc_set_duty_and_update(ledc_ch_cool.speed_mode, ledc_ch_cool.channel, cool_width, warm_width));
    }

    pwm_nonoverlap_current_warm = warm_width;
    pwm_nonoverlap_current_cool = cool_width;
}

static void pwm_nonoverlap_set_target(uint32_t warm_target, uint32_t cool_target, uint32_t target_time_ms)
{
    if (pwm_nonoverlap_mutex == NULL) {
        pwm_set_warm_cool_nonoverlap(warm_target, cool_target);
        return;
    }

    xSemaphoreTake(pwm_nonoverlap_mutex, portMAX_DELAY);

    warm_target = MIN(warm_target, PWM_MAX_DUTY);
    cool_target = MIN(cool_target, PWM_MAX_DUTY);
    if ((warm_target + cool_target) > PWM_MAX_DUTY) {
        if (warm_target >= PWM_MAX_DUTY) {
            warm_target = PWM_MAX_DUTY;
            cool_target = 0;
        } else {
            cool_target = PWM_MAX_DUTY - warm_target;
        }
    }

    uint32_t delta_warm = (pwm_nonoverlap_current_warm > warm_target)
                              ? (pwm_nonoverlap_current_warm - warm_target)
                              : (warm_target - pwm_nonoverlap_current_warm);
    uint32_t delta_cool = (pwm_nonoverlap_current_cool > cool_target)
                              ? (pwm_nonoverlap_current_cool - cool_target)
                              : (cool_target - pwm_nonoverlap_current_cool);
    uint32_t delta_max = MAX(delta_warm, delta_cool);

    if (delta_max == 0) {
        pwm_nonoverlap_ramp.active = false;
        xSemaphoreGive(pwm_nonoverlap_mutex);
        return;
    }

    uint64_t total_cycles = ((uint64_t)ledc_timer_cfg.freq_hz * (uint64_t)target_time_ms) / 1000ULL;
    uint32_t fade_step = PWM_FADE_STEP;
    if (warm_target == 0 && cool_target == 0) {
        fade_step = PWM_FADE_OFF_STEP;
    }
    if (fade_step == 0) {
        fade_step = 1;
    }
    uint32_t steps_required = (delta_max + fade_step - 1U) / fade_step;
    if (steps_required < 1U) {
        steps_required = 1U;
    }
    if (total_cycles < (uint64_t)steps_required) {
        total_cycles = (uint64_t)steps_required;
    }
    int64_t duration_us = (int64_t)((total_cycles * 1000000ULL) / (uint64_t)ledc_timer_cfg.freq_hz);
    if (duration_us < 1) {
        duration_us = 1;
    }

    pwm_nonoverlap_ramp.start_warm  = pwm_nonoverlap_current_warm;
    pwm_nonoverlap_ramp.start_cool  = pwm_nonoverlap_current_cool;
    pwm_nonoverlap_ramp.target_warm = warm_target;
    pwm_nonoverlap_ramp.target_cool = cool_target;
    pwm_nonoverlap_ramp.start_us    = esp_timer_get_time();
    pwm_nonoverlap_ramp.duration_us = duration_us;
    pwm_nonoverlap_ramp.active      = true;

    xSemaphoreGive(pwm_nonoverlap_mutex);
}

static void pwm_nonoverlap_timer_cb(void *arg)
{
    (void)arg;

    if (pwm_nonoverlap_mutex == NULL) {
        return;
    }
    if (xSemaphoreTake(pwm_nonoverlap_mutex, 0) != pdTRUE) {
        return;
    }
    if (!pwm_nonoverlap_ramp.active) {
        xSemaphoreGive(pwm_nonoverlap_mutex);
        return;
    }

    int64_t now_us = esp_timer_get_time();
    int64_t elapsed_us = now_us - pwm_nonoverlap_ramp.start_us;
    int64_t duration_us = pwm_nonoverlap_ramp.duration_us;

    if (duration_us <= 0 || elapsed_us >= duration_us) {
        pwm_set_warm_cool_nonoverlap(pwm_nonoverlap_ramp.target_warm, pwm_nonoverlap_ramp.target_cool);
        pwm_nonoverlap_ramp.active = false;
        xSemaphoreGive(pwm_nonoverlap_mutex);
        return;
    }

    int64_t warm_delta = (int64_t)pwm_nonoverlap_ramp.target_warm - (int64_t)pwm_nonoverlap_ramp.start_warm;
    int64_t cool_delta = (int64_t)pwm_nonoverlap_ramp.target_cool - (int64_t)pwm_nonoverlap_ramp.start_cool;
    uint32_t warm_next = (uint32_t)((int64_t)pwm_nonoverlap_ramp.start_warm + (warm_delta * elapsed_us) / duration_us);
    uint32_t cool_next = (uint32_t)((int64_t)pwm_nonoverlap_ramp.start_cool + (cool_delta * elapsed_us) / duration_us);

    pwm_set_warm_cool_nonoverlap(warm_next, cool_next);
    xSemaphoreGive(pwm_nonoverlap_mutex);
}
#endif

// ---------------------------- State variables (mirrors Arduino sketch) ---------
static bool  led_state          = false;
static float brightness_percent = 50.0f; // 1 … 100
static float temp_ratio         = 0.5f;  // 0.0 warm … 1.0 cool

#if CONFIG_OTL_CIRCADIAN_ENABLE
static float circadian_base_ratio = 0.5f;     // 0.0 warm .. 1.0 cool
static float temp_ratio_user_offset = 0.0f;   // applied on top of circadian_base_ratio
#endif

static const int   BRIGHT_STEP      = 1;
static const float TEMP_STEP        = 0.015f;
static const int   DOUBLE_TAP_MS    = 300;
// Perceptual brightness gamma (lower = brighter/less aggressive low-end dimming)
static const float GAMMA_CORRECTION = 1.6f;
// Limit maximum brightness to avoid overheating
static const int   MAX_BRIGHTNESS_PERCENT = 95;
static const float MIN_BRIGHTNESS_PERCENT = 5.0f;

// ---------------------------- Thermal protection ------------------------------
// If either sensor crosses the "hot" threshold, clamp the *effective* maximum
// brightness down until it cools by a small hysteresis.
#define OTL_THERMAL_NTC_HOT_C                    95.0f
#define OTL_THERMAL_NTC_HYST_C                   3.0f
#define OTL_THERMAL_CHIP_HOT_C                   75.0f
#define OTL_THERMAL_CHIP_HYST_C                  3.0f
#define OTL_THERMAL_DIM_MAX_BRIGHTNESS_PERCENT   30.0f
#define OTL_THERMAL_NTC_HOT_CONFIRM_POLLS        2
#define OTL_NTC_SAMPLE_COUNT                     5

static volatile bool thermal_ntc_hot = false;
static volatile bool thermal_chip_hot = false;
static volatile float thermal_brightness_cap_percent = 100.0f;
static uint8_t thermal_ntc_hot_confirm_polls = 0;
// While holding dim-down, allow a deeper range below the normal minimum.
// HOLD_MIN_BRIGHTNESS_PERCENT is the internal floor used for holds; the mapping
// below converts that to a physical PWM duty floor (HOLD_MIN_PWM_DUTY_RATIO).
static const float HOLD_MIN_BRIGHTNESS_PERCENT = 0.0f;
static const float HOLD_MIN_PWM_DUTY_RATIO     = (float)OTL_PWM_MIN_ON_DUTY / (float)OTL_PWM_MAX_DUTY;
// Extra offset added to brightness_percent before gamma, to avoid an overly-dim
// low end (e.g. 1% becomes ~15% linear before gamma).
static const int   MIN_BRIGHTNESS_OFFSET_PERCENT = 14;

#define POWER_HOLD_NEOPIXEL_MS 3000

// When holding brightness up/down, step the *total* PWM duty by exactly 1 count
// per tick so the combined output doesn't "jump" by multiple duty counts.
static esp_timer_handle_t pwm_hold_step_timer = NULL;
static volatile bool pwm_hold_stepper_enabled = false;
static volatile uint32_t pwm_hold_target_total_duty = 0;
static uint32_t pwm_hold_current_total_duty = 0;

#if CONFIG_OTL_SERIAL_OUTPUT && CONFIG_OTL_LOG_PWM_DUTY
static esp_timer_handle_t pwm_log_timer = NULL;
#endif
static SemaphoreHandle_t otl_state_mutex = NULL;
static bool radar_presence = false;

static inline float clampf(float v, float lo, float hi);

#if CONFIG_OTL_CIRCADIAN_ENABLE
static void circadian_apply_current_ratio(void)
{
    circadian_base_ratio = clampf(circadian_base_ratio, 0.0f, 1.0f);
    float min_offset = -circadian_base_ratio;
    float max_offset = 1.0f - circadian_base_ratio;
    temp_ratio_user_offset = clampf(temp_ratio_user_offset, min_offset, max_offset);
    temp_ratio = clampf(circadian_base_ratio + temp_ratio_user_offset, 0.0f, 1.0f);
}
#endif

static void otl_set_temp_ratio(float ratio)
{
    ratio = clampf(ratio, 0.0f, 1.0f);
#if CONFIG_OTL_CIRCADIAN_ENABLE
    temp_ratio_user_offset = ratio - circadian_base_ratio;
    circadian_apply_current_ratio();
#else
    temp_ratio = ratio;
#endif
}

static uint32_t brightness_percent_to_total_duty(float b_percent)
{
    float b = b_percent;
    if (b < HOLD_MIN_BRIGHTNESS_PERCENT) b = HOLD_MIN_BRIGHTNESS_PERCENT;
    if (b > (float)MAX_BRIGHTNESS_PERCENT) b = (float)MAX_BRIGHTNESS_PERCENT;

    float gamma_scaled = 0.0f;
    if (b < MIN_BRIGHTNESS_PERCENT) {
        float linear_min = (MIN_BRIGHTNESS_PERCENT + (float)MIN_BRIGHTNESS_OFFSET_PERCENT) / 100.0f;
        linear_min = clampf(linear_min, 0.0f, 1.0f);
        float duty_at_min = powf(linear_min, GAMMA_CORRECTION);

        float t = b / MIN_BRIGHTNESS_PERCENT; // 0..1
        t = clampf(t, 0.0f, 1.0f);
        gamma_scaled = HOLD_MIN_PWM_DUTY_RATIO + t * (duty_at_min - HOLD_MIN_PWM_DUTY_RATIO);
    } else {
        float linear = (b + (float)MIN_BRIGHTNESS_OFFSET_PERCENT) / 100.0f;
        linear = clampf(linear, 0.0f, 1.0f);
        gamma_scaled = powf(linear, GAMMA_CORRECTION);
    }

    float max_gamma_scaled = (float)MAX_BRIGHTNESS_PERCENT / 100.0f;
    if (gamma_scaled > max_gamma_scaled) {
        gamma_scaled = max_gamma_scaled;
    }

    uint32_t duty = (uint32_t)(gamma_scaled * PWM_MAX_DUTY);
    if (duty != 0 && duty < PWM_MIN_ON_DUTY) {
        duty = PWM_MIN_ON_DUTY;
    }
    return duty;
}

static float brightness_percent_from_total_duty(uint32_t total_duty)
{
    float lo = HOLD_MIN_BRIGHTNESS_PERCENT;
    float hi = (float)MAX_BRIGHTNESS_PERCENT;

    uint32_t duty_lo = brightness_percent_to_total_duty(lo);
    if (total_duty <= duty_lo) {
        return lo;
    }
    uint32_t duty_hi = brightness_percent_to_total_duty(hi);
    if (total_duty >= duty_hi) {
        return hi;
    }

    for (int i = 0; i < 24; ++i) {
        float mid = (lo + hi) * 0.5f;
        uint32_t mid_duty = brightness_percent_to_total_duty(mid);
        if (mid_duty < total_duty) {
            lo = mid;
        } else {
            hi = mid;
        }
    }

    uint32_t lo_duty = brightness_percent_to_total_duty(lo);
    uint32_t hi_duty = brightness_percent_to_total_duty(hi);
    if ((total_duty - lo_duty) <= (hi_duty - total_duty)) {
        return lo;
    }
    return hi;
}

static void pwm_hold_stepper_sync_from_hw(void)
{
#if CONFIG_OTL_NONOVERLAP_PWM
    pwm_hold_current_total_duty = pwm_nonoverlap_current_warm + pwm_nonoverlap_current_cool;
#else
    uint32_t warm = ledc_get_duty(ledc_ch_warm.speed_mode, ledc_ch_warm.channel);
    uint32_t cool = ledc_get_duty(ledc_ch_cool.speed_mode, ledc_ch_cool.channel);
    pwm_hold_current_total_duty = warm + cool;
#endif
    if (pwm_hold_current_total_duty > PWM_MAX_DUTY) {
        pwm_hold_current_total_duty = PWM_MAX_DUTY;
    }
}

static void pwm_hold_stepper_apply_total(uint32_t total_duty)
{
    if (total_duty > PWM_MAX_DUTY) {
        total_duty = PWM_MAX_DUTY;
    }

    float ratio = temp_ratio;
    if (ratio < 0.0f) ratio = 0.0f;
    if (ratio > 1.0f) ratio = 1.0f;

    uint32_t cool_duty = (uint32_t)((float)total_duty * ratio);
    if (cool_duty > total_duty) {
        cool_duty = total_duty;
    }
    uint32_t warm_duty = total_duty - cool_duty;

#if CONFIG_OTL_NONOVERLAP_PWM
    pwm_set_warm_cool_nonoverlap(warm_duty, cool_duty);
#else
    pwm_set_warm_cool_overlap(warm_duty, cool_duty);
#endif
}

static void pwm_hold_stepper_timer_cb(void *arg)
{
    (void)arg;
    if (!pwm_hold_stepper_enabled || !led_state) {
        return;
    }

    uint32_t target = pwm_hold_target_total_duty;
    if (pwm_hold_current_total_duty == target) {
        return;
    }

    if (pwm_hold_current_total_duty < target) {
        pwm_hold_current_total_duty++;
    } else {
        pwm_hold_current_total_duty--;
    }

    pwm_hold_stepper_apply_total(pwm_hold_current_total_duty);
}

static void pwm_hold_stepper_set_enabled(bool enabled)
{
    if (enabled == pwm_hold_stepper_enabled) {
        return;
    }

    if (enabled) {
        // Cancel any in-progress fades/ramp so the stepper is the sole writer.
        (void)ledc_fade_stop(ledc_ch_warm.speed_mode, ledc_ch_warm.channel);
        (void)ledc_fade_stop(ledc_ch_cool.speed_mode, ledc_ch_cool.channel);
#if CONFIG_OTL_NONOVERLAP_PWM
        pwm_nonoverlap_ramp.active = false;
#endif
        pwm_hold_stepper_sync_from_hw();
        pwm_hold_stepper_enabled = true;
        ESP_ERROR_CHECK(esp_timer_start_periodic(pwm_hold_step_timer, PWM_HOLD_STEP_UPDATE_US));
    } else {
        pwm_hold_stepper_enabled = false;
        (void)esp_timer_stop(pwm_hold_step_timer);
        if (led_state) {
            pwm_hold_stepper_sync_from_hw();
            pwm_hold_target_total_duty = pwm_hold_current_total_duty;
            float b_cap = clampf((float)thermal_brightness_cap_percent, HOLD_MIN_BRIGHTNESS_PERCENT, (float)MAX_BRIGHTNESS_PERCENT);
            if (brightness_percent <= (b_cap + 0.01f)) {
                brightness_percent = brightness_percent_from_total_duty(pwm_hold_current_total_duty);
            }
        }
    }
}

#if CONFIG_OTL_SERIAL_OUTPUT && CONFIG_OTL_LOG_PWM_DUTY
static void pwm_log_timer_cb(void *arg)
{
    (void)arg;

    uint32_t warm = 0;
    uint32_t cool = 0;

#if CONFIG_OTL_NONOVERLAP_PWM
    warm = pwm_nonoverlap_current_warm;
    cool = pwm_nonoverlap_current_cool;
#else
    warm = ledc_get_duty(ledc_ch_warm.speed_mode, ledc_ch_warm.channel);
    cool = ledc_get_duty(ledc_ch_cool.speed_mode, ledc_ch_cool.channel);
#endif

    if (warm == 0 && cool == 0) {
        return;
    }

    uint32_t total = warm + cool;
    if (total > PWM_MAX_DUTY) {
        total = PWM_MAX_DUTY;
    }

    uint32_t target_total = led_state ? pwm_hold_target_total_duty : 0;

    static uint32_t last_warm = 0xFFFFFFFFu;
    static uint32_t last_cool = 0xFFFFFFFFu;
    static uint32_t last_target_total = 0xFFFFFFFFu;
    static bool last_led_state = false;

    if (warm == last_warm &&
        cool == last_cool &&
        target_total == last_target_total &&
        led_state == last_led_state) {
        return;
    }

    last_warm = warm;
    last_cool = cool;
    last_target_total = target_total;
    last_led_state = led_state;

    OTL_LOGI_PERIODIC("pwm",
                      "state=%s B=%.1f%% temp=%.3f warm=%lu cool=%lu total=%lu target=%lu stepper=%d",
                      led_state ? "ON" : "OFF",
                      brightness_percent,
                      temp_ratio,
                      (unsigned long)warm,
                      (unsigned long)cool,
                      (unsigned long)total,
                      (unsigned long)target_total,
                      pwm_hold_stepper_enabled ? 1 : 0);
}
#endif

#if CONFIG_OTL_PRESENCE_SENSOR
// ---------------------------- Occupancy (LD2410B) ----------------------------
// The LD2410B's digital OUT pin drives this occupancy state.
// We use it to fade the light off when the room is empty, and
// fade back in to the previous state when occupancy returns.
static bool radar_occupied        = false;
static bool occupancy_faded_out   = false;
static bool saved_led_state       = false;
static int  saved_brightness      = 0;
#if CONFIG_OTL_CIRCADIAN_ENABLE
static float saved_temp_ratio_offset = 0.0f;
#else
static float saved_temp_ratio     = 0.5f;
#endif
// Detection and hysteresis tuning from menuconfig -> Open Task Light -> Radar detection.
#define RADAR_MOVING_NEAR_MAX_CM      ((uint16_t)CONFIG_OTL_RADAR_MOVING_MAX_DISTANCE_CM)
#define RADAR_PRESENCE_MAX_DIST_CM    ((uint16_t)CONFIG_OTL_RADAR_STATIONARY_MAX_DISTANCE_CM)
#define RADAR_PRESENCE_ON_DELAY_MS    ((uint32_t)CONFIG_OTL_RADAR_PRESENCE_ON_DELAY_MS)
#define RADAR_ABSENCE_TIMEOUT_MS      ((uint32_t)CONFIG_OTL_RADAR_ABSENCE_TIMEOUT_MS)
#define RADAR_TASK_LOOP_INTERVAL_MS   ((uint32_t)CONFIG_OTL_RADAR_TASK_LOOP_MS)
#endif

// ---------------------------- Touch baseline/threshold -------------------------
#define PAD_COUNT 5
static const touch_pad_t touch_pads[PAD_COUNT] = {
    TOUCH_LED_SWITCH,
    TOUCH_SWITCH_UP,
    TOUCH_SWITCH_DN,
    TOUCH_TEMP_UP,
    TOUCH_TEMP_DN
};
static uint32_t baseline[PAD_COUNT];
static uint32_t threshold[PAD_COUNT];
// Touch thresholds are computed as:
//   threshold = baseline + clamp(max(baseline * pct, noise_span * mult), min, max)
// This is more robust across different power/ground conditions than a pure
// baseline*factor threshold (which can become too strict when baseline rises).
#define TOUCH_CAL_SAMPLES                 120
#define TOUCH_CAL_SAMPLE_DELAY_MS         2
#define TOUCH_NOISE_MULTIPLIER            6U
#define TOUCH_THRESHOLD_MARGIN_MIN        250U
#define TOUCH_THRESHOLD_MARGIN_MAX        2500U
#define TOUCH_CAL_EDGE_SAMPLE_WINDOW      8
#define TOUCH_CAL_MIN_VALID_SAMPLES       80
#define TOUCH_CAL_MAX_DRIFT_MIN           300U
static const float touch_threshold_margin_pct[PAD_COUNT] = { 0.05f, 0.03f, 0.03f, 0.03f, 0.03f };

typedef struct {
    uint8_t low_valid_pads;
    uint8_t drifting_pads;
} touch_calibration_result_t;

static void touch_read_raw_snapshot(uint32_t raw[PAD_COUNT])
{
    for (int i = 0; i < PAD_COUNT; ++i) {
        touch_pad_read_raw_data(touch_pads[i], &raw[i]);
    }
}

static int touch_count_pressed_from_raw(const uint32_t raw[PAD_COUNT])
{
    int pressed_count = 0;

    for (int i = 0; i < PAD_COUNT; ++i) {
        if (raw[i] > threshold[i]) {
            pressed_count++;
        }
    }

    return pressed_count;
}

static bool touch_confirm_pressed_count_at_least(int min_pressed)
{
    for (int sample = 0; sample < TOUCH_VERIFY_SAMPLES; ++sample) {
        uint32_t raw[PAD_COUNT] = {0};
        touch_read_raw_snapshot(raw);
        if (touch_count_pressed_from_raw(raw) < min_pressed) {
            return false;
        }
        if (sample + 1 < TOUCH_VERIFY_SAMPLES) {
            vTaskDelay(pdMS_TO_TICKS(TOUCH_VERIFY_DELAY_MS));
        }
    }

    return true;
}

static touch_calibration_result_t calibrate_touch_pads(void)
{
    touch_calibration_result_t result = {0};

    OTL_LOG_TOUCH_CALI("Calibrating touch pads (dynamic baseline/threshold)...");

    // Strategy adapted from the Arduino sketch:
    //  - sample each pad several times with no touch
    //  - compute a baseline per pad
    //  - set a per-pad threshold as baseline * factor[i]
    //  - on this board, touch makes the value RISE,
    //    so we treat \"value ABOVE threshold\" as a touch
    for (int i = 0; i < PAD_COUNT; ++i) {
        uint64_t sum = 0;
        uint64_t first_sum = 0;
        uint64_t last_sum = 0;
        int valid_samples = 0;
        int first_count = 0;
        int last_count = 0;
        int last_index = 0;
        uint32_t min_val = 0xFFFFFFFFu;
        uint32_t max_val = 0;
        uint32_t last_samples[TOUCH_CAL_EDGE_SAMPLE_WINDOW] = {0};

        for (int s = 0; s < TOUCH_CAL_SAMPLES; ++s) {
            uint32_t val;
            touch_pad_read_raw_data(touch_pads[i], &val);
            // Some pads can briefly report 0 right after start-up.
            // Ignore those so we don't get a bogus baseline of 0.
            if (val != 0) {
                sum += val;
                valid_samples++;
                if (first_count < TOUCH_CAL_EDGE_SAMPLE_WINDOW) {
                    first_sum += val;
                    first_count++;
                }
                if (last_count < TOUCH_CAL_EDGE_SAMPLE_WINDOW) {
                    last_sum += val;
                    last_samples[last_count++] = val;
                } else {
                    last_sum -= last_samples[last_index];
                    last_samples[last_index] = val;
                    last_sum += val;
                    last_index = (last_index + 1) % TOUCH_CAL_EDGE_SAMPLE_WINDOW;
                }
                if (val < min_val) min_val = val;
                if (val > max_val) max_val = val;
            }
            vTaskDelay(pdMS_TO_TICKS(TOUCH_CAL_SAMPLE_DELAY_MS));
        }

        if (valid_samples == 0) {
            // Fallback: avoid a zero baseline; pick a small non-zero default
            baseline[i] = 1000;
            min_val = baseline[i];
            max_val = baseline[i];
        } else {
            baseline[i] = sum / valid_samples;
        }

        uint32_t noise_span = (max_val > min_val && min_val != 0xFFFFFFFFu) ? (max_val - min_val) : 0;
        uint32_t margin_pct = (uint32_t)((float)baseline[i] * touch_threshold_margin_pct[i]);
        uint32_t margin_noise = noise_span * TOUCH_NOISE_MULTIPLIER;
        uint32_t margin = (margin_pct > margin_noise) ? margin_pct : margin_noise;
        margin = clamp_u32(margin, TOUCH_THRESHOLD_MARGIN_MIN, TOUCH_THRESHOLD_MARGIN_MAX);
        threshold[i] = baseline[i] + margin;

        uint32_t drift = 0;
        if (first_count > 0 && last_count > 0) {
            uint32_t first_avg = (uint32_t)(first_sum / (uint64_t)first_count);
            uint32_t last_avg = (uint32_t)(last_sum / (uint64_t)last_count);
            drift = (first_avg > last_avg) ? (first_avg - last_avg) : (last_avg - first_avg);
            uint32_t allowed_drift = MAX(TOUCH_CAL_MAX_DRIFT_MIN, baseline[i] / 100U);
            if (drift > allowed_drift) {
                result.drifting_pads++;
            }
        }
        if (valid_samples < TOUCH_CAL_MIN_VALID_SAMPLES) {
            result.low_valid_pads++;
        }

        ESP_ERROR_CHECK(touch_pad_set_thresh(touch_pads[i], threshold[i]));

        OTL_LOG_TOUCH_CALI("Pad %d baseline=%lu threshold=%lu noise_span=%lu margin=%lu drift=%lu valid=%d",
                 i,
                 (unsigned long)baseline[i],
                 (unsigned long)threshold[i],
                 (unsigned long)noise_span,
                 (unsigned long)margin,
                 (unsigned long)drift,
                 valid_samples);
    }

    return result;
}

static void touch_calibrate_startup(void)
{
    for (int attempt = 1; attempt <= TOUCH_STARTUP_MAX_RETRIES; ++attempt) {
        touch_calibration_result_t cal = calibrate_touch_pads();
        bool bad_calibration = (cal.low_valid_pads > 0) ||
                               (cal.drifting_pads > 0) ||
                               touch_confirm_pressed_count_at_least(TOUCH_BAD_PRESSED_PADS);

        if (!bad_calibration) {
            if (attempt > 1) {
                OTL_LOGI("touch", "Startup calibration stabilized on attempt %d", attempt);
            }
            return;
        }

        OTL_LOGW("touch",
                 "Startup calibration retry %d/%d (low_valid=%u drifting=%u)",
                 attempt,
                 TOUCH_STARTUP_MAX_RETRIES,
                 cal.low_valid_pads,
                 cal.drifting_pads);

        if (attempt < TOUCH_STARTUP_MAX_RETRIES) {
            vTaskDelay(pdMS_TO_TICKS(TOUCH_STARTUP_RETRY_MS));
        }
    }
}

static void recalibration_task(void *arg)
{
    (void)arg;
    vTaskDelay(pdMS_TO_TICKS(TOUCH_RECOVERY_DELAY_MS));

    while (1) {
        if (touch_confirm_pressed_count_at_least(TOUCH_BAD_PRESSED_PADS)) {
            OTL_LOGW("touch", "Multiple pads appear pressed at idle; forcing recalibration");
            calibrate_touch_pads();
        } else {
            // Only recalibrate if no buttons are being touched
            bool can_recalibrate = true;
            uint32_t raw[PAD_COUNT] = {0};
            touch_read_raw_snapshot(raw);
            for (int i = 0; i < PAD_COUNT; ++i) {
                if (raw[i] > threshold[i]) {
                    can_recalibrate = false;
                    break;
                }
            }

            if (can_recalibrate) {
                OTL_LOG_TOUCH_CALI("Performing periodic recalibration");
                calibrate_touch_pads();
            }
        }

        // Wait for a period with no touch activity
        vTaskDelay(pdMS_TO_TICKS(RECALIB_INTERVAL_MS));
    }
}

// ---------------------------- Radar UART frame parsing ------------------------
#if CONFIG_OTL_PRESENCE_SENSOR
// Parse incoming LD2410 frames into radar_last_sample, similar to LD2410B.ino.
static void radar_parse_frames(void)
{
    size_t i = 0;

    while (i + 10 <= radar_rx_len) {
        // Find header F4 F3 F2 F1
        if (!(radar_rx_buf[i] == 0xF4 &&
              radar_rx_buf[i + 1] == 0xF3 &&
              radar_rx_buf[i + 2] == 0xF2 &&
              radar_rx_buf[i + 3] == 0xF1)) {
            ++i;
            continue;
        }
        if (i + 8 > radar_rx_len) break; // need at least header+len

        uint16_t L = (uint16_t)radar_rx_buf[i + 4] |
                     ((uint16_t)radar_rx_buf[i + 5] << 8);
        size_t frameLen = (size_t)L + 10; // 4 header + 2 len + L payload + 4 tail
        if (frameLen > sizeof(radar_rx_buf)) {
            // drop this byte and resync
            ++i;
            continue;
        }
        if (i + frameLen > radar_rx_len) break; // wait for more

        // Verify tail F8 F7 F6 F5
        if (!(radar_rx_buf[i + frameLen - 4] == 0xF8 &&
              radar_rx_buf[i + frameLen - 3] == 0xF7 &&
              radar_rx_buf[i + frameLen - 2] == 0xF6 &&
              radar_rx_buf[i + frameLen - 1] == 0xF5)) {
            ++i; // bad alignment; resync
            continue;
        }

        // Got a full data frame
        const uint8_t *f = &radar_rx_buf[i];

        radar_sample_t s = {0};
        // Basic LD2410 data frame layout as in the Arduino sketch
        if (L >= 13 && f[6] == 0x02 && f[7] == 0xAA && f[17] == 0x55 && f[18] == 0x00) {
            s.valid       = true;
            s.targetType  = f[8];
            s.movingDist  = (uint16_t)f[9] | ((uint16_t)f[10] << 8);
            s.movingEnergy = f[11];
            s.staticDist   = (uint16_t)f[12] | ((uint16_t)f[13] << 8);
            s.staticEnergy = f[14];
            // Clamp energies to 0-100 for readability
            if (s.movingEnergy > RADAR_MAX_ENERGY) s.movingEnergy = RADAR_MAX_ENERGY;
            if (s.staticEnergy > RADAR_MAX_ENERGY) s.staticEnergy = RADAR_MAX_ENERGY;
        }

        if (s.valid) {
            radar_last_sample = s;
        }

        // Consume this frame from the buffer
        size_t remain = radar_rx_len - (i + frameLen);
        if (remain) {
            memmove(radar_rx_buf, &radar_rx_buf[i + frameLen], remain);
        }
        radar_rx_len = remain;
        i = 0; // restart from beginning after compaction
    }

    // If we skipped ahead without consuming a frame, discard leading noise
    if (i > 0 && i < radar_rx_len) {
        memmove(radar_rx_buf, &radar_rx_buf[i], radar_rx_len - i);
        radar_rx_len -= i;
    } else if (i >= radar_rx_len) {
        radar_rx_len = 0;
    }

}
#endif

// ---------------------------- Helper functions ---------------------------------
static void ws2812_set_rgb(uint8_t r, uint8_t g, uint8_t b)
{
    led_strip_set_pixel(strip, 0, r, g, b);
    led_strip_refresh(strip);
}

// Forward declaration so occupancy helpers can call it.
static void update_outputs(void);

#define OTL_MAX_STATE_LISTENERS 4

typedef struct {
    otl_state_listener_fn listener;
    void *ctx;
} otl_state_listener_entry_t;

static otl_state_listener_entry_t otl_state_listeners[OTL_MAX_STATE_LISTENERS];

static void otl_state_lock(void)
{
    xSemaphoreTake(otl_state_mutex, portMAX_DELAY);
}

static void otl_state_unlock(void)
{
    xSemaphoreGive(otl_state_mutex);
}

static bool otl_state_float_changed(float a, float b)
{
    return fabsf(a - b) > 0.0005f;
}

static void otl_state_snapshot_locked(otl_public_state_t *state)
{
    if (state == NULL) {
        return;
    }

    state->is_on = led_state;
    state->brightness_percent = brightness_percent;
    state->temp_ratio = temp_ratio;
    state->presence = radar_presence;
}

static void otl_state_notify_listeners(const otl_public_state_t *state,
                                       otl_change_source_t source)
{
    otl_state_listener_entry_t listeners[OTL_MAX_STATE_LISTENERS] = {0};
    size_t listener_count = 0;

    otl_state_lock();
    for (size_t i = 0; i < OTL_MAX_STATE_LISTENERS; ++i) {
        if (otl_state_listeners[i].listener == NULL) {
            continue;
        }
        listeners[listener_count++] = otl_state_listeners[i];
    }
    otl_state_unlock();

    for (size_t i = 0; i < listener_count; ++i) {
        listeners[i].listener(state, source, listeners[i].ctx);
    }
}

void otl_state_get_public(otl_public_state_t *state)
{
    if (state == NULL) {
        return;
    }

    otl_state_lock();
    otl_state_snapshot_locked(state);
    otl_state_unlock();
}

bool otl_state_apply_light_update(const otl_light_update_t *update,
                                  otl_change_source_t source)
{
    bool changed = false;
    otl_public_state_t state = {0};

    if (update == NULL) {
        return false;
    }

    otl_state_lock();

    if (update->set_power && led_state != update->power_on) {
        changed = true;
        led_state = update->power_on;
    }

    if (update->set_brightness) {
        float new_brightness = clampf(update->brightness_percent,
                                      HOLD_MIN_BRIGHTNESS_PERCENT,
                                      (float)MAX_BRIGHTNESS_PERCENT);
        if (otl_state_float_changed(brightness_percent, new_brightness)) {
            changed = true;
            brightness_percent = new_brightness;
        }
        if (update->force_on_with_brightness && !led_state) {
            changed = true;
            led_state = true;
        }
    }

    if (update->set_temp_ratio) {
        float prev_temp_ratio = temp_ratio;
        otl_set_temp_ratio(update->temp_ratio);
        if (otl_state_float_changed(prev_temp_ratio, temp_ratio)) {
            changed = true;
        }
    }

    if (!changed) {
        otl_state_unlock();
        return false;
    }

    update_outputs();
    otl_state_snapshot_locked(&state);
    otl_state_unlock();
    otl_state_notify_listeners(&state, source);
    return true;
}

bool otl_state_set_presence(bool presence, otl_change_source_t source)
{
    bool changed = false;
    otl_public_state_t state = {0};

    otl_state_lock();
    if (radar_presence != presence) {
        radar_presence = presence;
        changed = true;
        otl_state_snapshot_locked(&state);
    }
    otl_state_unlock();

    if (changed) {
        otl_state_notify_listeners(&state, source);
    }

    return changed;
}

esp_err_t otl_state_add_listener(otl_state_listener_fn listener, void *ctx)
{
    if (listener == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    otl_state_lock();
    for (size_t i = 0; i < OTL_MAX_STATE_LISTENERS; ++i) {
        if (otl_state_listeners[i].listener != NULL) {
            continue;
        }
        otl_state_listeners[i].listener = listener;
        otl_state_listeners[i].ctx = ctx;
        otl_state_unlock();
        return ESP_OK;
    }
    otl_state_unlock();
    return ESP_ERR_NO_MEM;
}

void otl_state_notify_current(otl_change_source_t source)
{
    otl_public_state_t state = {0};
    otl_state_get_public(&state);
    otl_state_notify_listeners(&state, source);
}

static void thermal_update(float ntc_c, float chip_c)
{
    bool prev_ntc_hot = false;
    bool prev_chip_hot = false;
    bool throttled = false;
    float new_cap = (float)MAX_BRIGHTNESS_PERCENT;

    otl_state_lock();
    prev_ntc_hot = thermal_ntc_hot;
    prev_chip_hot = thermal_chip_hot;

    if (!isnan(ntc_c)) {
        if (!thermal_ntc_hot) {
            if (ntc_c >= OTL_THERMAL_NTC_HOT_C) {
                if (thermal_ntc_hot_confirm_polls < 255) {
                    thermal_ntc_hot_confirm_polls++;
                }
                if (thermal_ntc_hot_confirm_polls >= OTL_THERMAL_NTC_HOT_CONFIRM_POLLS) {
                    thermal_ntc_hot = true;
                    thermal_ntc_hot_confirm_polls = 0;
                }
            } else {
                thermal_ntc_hot_confirm_polls = 0;
            }
        } else {
            thermal_ntc_hot_confirm_polls = 0;
            if (ntc_c <= (OTL_THERMAL_NTC_HOT_C - OTL_THERMAL_NTC_HYST_C)) {
                thermal_ntc_hot = false;
            }
        }
    } else {
        thermal_ntc_hot_confirm_polls = 0;
    }

    if (!isnan(chip_c)) {
        if (!thermal_chip_hot) {
            if (chip_c >= OTL_THERMAL_CHIP_HOT_C) {
                thermal_chip_hot = true;
            }
        } else {
            if (chip_c <= (OTL_THERMAL_CHIP_HOT_C - OTL_THERMAL_CHIP_HYST_C)) {
                thermal_chip_hot = false;
            }
        }
    }

    throttled = thermal_ntc_hot || thermal_chip_hot;
    new_cap = throttled ? OTL_THERMAL_DIM_MAX_BRIGHTNESS_PERCENT : (float)MAX_BRIGHTNESS_PERCENT;

    if (fabsf(new_cap - (float)thermal_brightness_cap_percent) > 0.01f) {
        thermal_brightness_cap_percent = new_cap;
        if (led_state) {
            update_outputs();
        }
    }
    otl_state_unlock();

    if ((prev_ntc_hot != thermal_ntc_hot) || (prev_chip_hot != thermal_chip_hot)) {
        OTL_LOGW("thermal",
                 "Thermal %s (NTC=%s%.1fC, Chip=%s%.1fC) -> max=%.0f%%",
                 throttled ? "LIMIT" : "OK",
                 thermal_ntc_hot ? "HOT " : "",
                 ntc_c,
                 thermal_chip_hot ? "HOT " : "",
                 chip_c,
                 new_cap);
    }
}

#if CONFIG_OTL_CIRCADIAN_ENABLE
static void otl_circadian_apply_cb(float base_ratio, void *ctx)
{
    bool changed = false;
    otl_public_state_t state = {0};

    (void)ctx;
    otl_state_lock();
    float prev_temp_ratio = temp_ratio;
    circadian_base_ratio = base_ratio;
    circadian_apply_current_ratio();
    changed = otl_state_float_changed(prev_temp_ratio, temp_ratio);
    if (led_state && changed) {
        update_outputs();
    }
    if (changed) {
        otl_state_snapshot_locked(&state);
    }
    otl_state_unlock();
    if (changed) {
        otl_state_notify_listeners(&state, OTL_CHANGE_SOURCE_CIRCADIAN);
    }
}
#endif

#if CONFIG_OTL_PRESENCE_SENSOR
// ---------------------------- Occupancy helpers ----------------------------
// Simple fade utility used when occupancy changes.
static void fade_brightness(int start, int end, int step, int delay_ms)
{
    if (start == end || step <= 0) {
        return;
    }

    if (start < end) {
        for (int b = start; b <= end; b += step) {
            otl_state_apply_light_update(&(otl_light_update_t) {
                .set_brightness = true,
                .force_on_with_brightness = true,
                .brightness_percent = (float)b,
            }, OTL_CHANGE_SOURCE_OCCUPANCY);
            vTaskDelay(pdMS_TO_TICKS(delay_ms));
        }
    } else {
        for (int b = start; b >= end; b -= step) {
            otl_state_apply_light_update(&(otl_light_update_t) {
                .set_brightness = true,
                .force_on_with_brightness = true,
                .brightness_percent = (float)b,
            }, OTL_CHANGE_SOURCE_OCCUPANCY);
            vTaskDelay(pdMS_TO_TICKS(delay_ms));
        }
    }
}

static void handle_occupancy_lost(void)
{
    int fade_start = 0;

    otl_state_lock();
    radar_occupied = false;

    // Only fade out if the light is currently on and
    // we haven't already faded it out.
    if (!led_state || occupancy_faded_out) {
        otl_state_unlock();
        return;
    }

    saved_led_state  = led_state;
    saved_brightness = (int)lroundf(fminf(brightness_percent, (float)MAX_BRIGHTNESS_PERCENT));
    fade_start = (int)lroundf(brightness_percent);
#if CONFIG_OTL_CIRCADIAN_ENABLE
    saved_temp_ratio_offset = temp_ratio_user_offset;
#else
    saved_temp_ratio = temp_ratio;
#endif
    otl_state_unlock();

    // Fade down to 0% brightness, then turn off.
    fade_brightness(fade_start, 0, BRIGHT_STEP, 15);
    occupancy_faded_out = true;
    otl_state_apply_light_update(&(otl_light_update_t) {
        .set_power = true,
        .power_on = false,
    }, OTL_CHANGE_SOURCE_OCCUPANCY);
}

static void handle_occupancy_gained(void)
{
    int fade_end = 0;

    otl_state_lock();
    radar_occupied = true;

    // Only fade back in if we previously faded out the light
    // and the saved state indicates it should be on.
    if (!occupancy_faded_out || !saved_led_state) {
        otl_state_unlock();
        return;
    }

    fade_end = saved_brightness;
#if CONFIG_OTL_CIRCADIAN_ENABLE
    temp_ratio_user_offset = saved_temp_ratio_offset;
    circadian_apply_current_ratio();
#else
    temp_ratio = saved_temp_ratio;
#endif

    // Fade from 0 up to the saved brightness.
    brightness_percent = 0;
    led_state = true;
    otl_public_state_t state = {0};
    update_outputs();
    occupancy_faded_out = false;
    otl_state_snapshot_locked(&state);
    otl_state_unlock();
    otl_state_notify_listeners(&state, OTL_CHANGE_SOURCE_OCCUPANCY);
    fade_brightness(0, fade_end, BRIGHT_STEP, 15);
}
#endif

static void update_outputs(void)
{
    if (!led_state) {
        // Turn off all outputs
#if CONFIG_OTL_NONOVERLAP_PWM
        pwm_nonoverlap_set_target(0, 0, PWM_FADE_OFF_TIME_MS);
#else
        pwm_set_duty_smooth_timed(ledc_ch_cool.speed_mode, ledc_ch_cool.channel, 0, PWM_FADE_OFF_TIME_MS);
        pwm_set_duty_smooth_timed(ledc_ch_warm.speed_mode, ledc_ch_warm.channel, 0, PWM_FADE_OFF_TIME_MS);
#endif
        ws2812_set_rgb(0, 0, 0);
        return;
    }

    // Clamp brightness to thermal cap, then convert to PWM duty via gamma curve.
    float b_cap = clampf((float)thermal_brightness_cap_percent, HOLD_MIN_BRIGHTNESS_PERCENT, (float)MAX_BRIGHTNESS_PERCENT);
    float b = clampf(brightness_percent, HOLD_MIN_BRIGHTNESS_PERCENT, b_cap);
    uint32_t total_duty = brightness_percent_to_total_duty(b);
    pwm_hold_target_total_duty = total_duty;

    if (!pwm_hold_stepper_enabled) {
        uint32_t cool_duty  = (uint32_t)(total_duty * temp_ratio);
        uint32_t warm_duty  = total_duty - cool_duty;

#if CONFIG_OTL_NONOVERLAP_PWM
        pwm_nonoverlap_set_target(warm_duty, cool_duty, PWM_FADE_TARGET_TIME_MS);
#else
        pwm_set_duty_smooth(ledc_ch_cool.speed_mode, ledc_ch_cool.channel, cool_duty);
        pwm_set_duty_smooth(ledc_ch_warm.speed_mode, ledc_ch_warm.channel, warm_duty);
#endif
    }

    if (neopixel_enabled) {
        float duty_ratio = (float)total_duty / (float)PWM_MAX_DUTY;
        uint8_t ws_brightness = (uint8_t)(duty_ratio * 255.0f + 0.5f);
        uint8_t red  = (uint8_t)((1.0f - temp_ratio) * ws_brightness);
        uint8_t blue = (uint8_t)(temp_ratio * ws_brightness);
        ws2812_set_rgb(red, 0, blue);
    } else {
        ws2812_set_rgb(0, 0, 0);
    }
}

// ---------------------------- ADC helpers (ALS + NTC) --------------------------
static adc_oneshot_unit_handle_t adc1_handle;
static adc_oneshot_unit_handle_t adc2_handle;
static const adc_channel_t als_channel = ADC_CHANNEL_4;   // GPIO5
static const adc_channel_t ntc_channel = ADC_CHANNEL_5;   // GPIO16

#if SOC_TEMP_SENSOR_SUPPORTED
static temperature_sensor_handle_t chip_temp_handle = NULL;

static void chip_temp_init(void)
{
    // Pick a range that covers typical ESP32-S3 junction temperatures while
    // still allowing the driver to select a valid internal calibration range.
    temperature_sensor_config_t cfg = TEMPERATURE_SENSOR_CONFIG_DEFAULT(20, 100);

    esp_err_t err = temperature_sensor_install(&cfg, &chip_temp_handle);
    if (err != ESP_OK) {
        OTL_LOGW("sensor", "Chip temp sensor install failed: %s", esp_err_to_name(err));
        chip_temp_handle = NULL;
        return;
    }

    err = temperature_sensor_enable(chip_temp_handle);
    if (err != ESP_OK) {
        OTL_LOGW("sensor", "Chip temp sensor enable failed: %s", esp_err_to_name(err));
        (void)temperature_sensor_uninstall(chip_temp_handle);
        chip_temp_handle = NULL;
    }
}

static float read_chip_temperature(void)
{
    if (chip_temp_handle == NULL) {
        return NAN;
    }

    float celsius = NAN;
    if (temperature_sensor_get_celsius(chip_temp_handle, &celsius) != ESP_OK) {
        return NAN;
    }
    return celsius;
}
#else
static void chip_temp_init(void) {}
static float read_chip_temperature(void) { return NAN; }
#endif

static void adc_init(void)
{
    // ADC1 for ALS
    adc_oneshot_unit_init_cfg_t cfg1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&cfg1, &adc1_handle));

    adc_oneshot_chan_cfg_t chan_cfg = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten    = OTL_ADC_ATTEN
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, als_channel, &chan_cfg));

    // ADC2 for NTC
    adc_oneshot_unit_init_cfg_t cfg2 = {
        .unit_id = ADC_UNIT_2,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&cfg2, &adc2_handle));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc2_handle, ntc_channel, &chan_cfg));

    // Calibration for ADC2
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_2,
        .atten = OTL_ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_12,
    };
    ESP_ERROR_CHECK(adc_cali_create_scheme_curve_fitting(&cali_config, &adc2_cali_handle));
}

static float __attribute__((unused)) read_als_lux(void)
{
    int raw = 0;
    esp_err_t err = adc_oneshot_read(adc1_handle, als_channel, &raw);
    if (err != ESP_OK) {
#if CONFIG_OTL_SENSOR_DEBUG
        OTL_LOGW("sensor", "ALS read failed: %s", esp_err_to_name(err));
#endif
        return 0.0f;  // Return safe default on error
    }
    float voltage = (raw / 4095.0f) * 3.3f;
    float iph_uA  = (voltage / 10000.0f) * 1e6f;
    float lux = (iph_uA / 100.0f) * 1000.0f;
#if CONFIG_OTL_SENSOR_DEBUG
    OTL_LOGI_PERIODIC("sensor", "ALS raw=%d V=%.3fV Iph=%.1fuA lux=%.1f", raw, voltage, iph_uA, lux);
#endif
    return lux;
}

static int compare_float_asc(const void *lhs, const void *rhs)
{
    const float a = *(const float *)lhs;
    const float b = *(const float *)rhs;
    return (a > b) - (a < b);
}

static float read_ntc_temperature_sample(void)
{
    const float series_resistor = 10000.0f;
    const float nominal_res    = 10000.0f;
    const float nominal_temp   = 25.0f;
    const float b_coeff        = 3434.0f;

    int raw = 0;
    esp_err_t err = adc_oneshot_read(adc2_handle, ntc_channel, &raw);
    if (err != ESP_OK || raw == 0 || raw == 4095) {
        return NAN;
    }

    int voltage_mv = 0;
    err = adc_cali_raw_to_voltage(adc2_cali_handle, raw, &voltage_mv);
    if (err != ESP_OK) {
        return NAN;
    }
    float v = voltage_mv / 1000.0f;

    float r  = series_resistor * (v / (3.3f - v));

#if CONFIG_OTL_SENSOR_DEBUG
    OTL_LOGI_PERIODIC("sensor", "NTC raw=%d V=%.3fV R=%.1fohm", raw, v, r);
#endif

    if (r <= 0) return NAN;

    float st = logf(r / nominal_res) / b_coeff + 1.0f / (nominal_temp + 273.15f);
    return (1.0f / st) - 273.15f;
}

static float read_ntc_temperature(void)
{
    float samples[OTL_NTC_SAMPLE_COUNT] = {0};
    size_t valid_count = 0;

    for (size_t i = 0; i < OTL_NTC_SAMPLE_COUNT; ++i) {
        float sample = read_ntc_temperature_sample();
        if (!isnan(sample)) {
            samples[valid_count++] = sample;
        }
    }

    if (valid_count == 0) {
        return NAN;
    }

    qsort(samples, valid_count, sizeof(samples[0]), compare_float_asc);
    return samples[valid_count / 2];
}


// --- Timing constants ---
// Hold threshold and repeat interval in ms for B+/B-/T+/T-.
// Smaller HOLD_DELAY_MS makes hold kick in sooner, smaller REPEAT_MS
// makes adjustments faster while still using 1-step resolution.
#define TOUCH_LOOP_DELAY_MS 5
#define HOLD_DELAY_MS 90

// When holding B+/B-, we accelerate the adjustment rate. Near the min/max ends
// of the range, ease back off so the last few percent don't feel like a cliff.
#define BRIGHTNESS_HOLD_SLOW_ZONE_PERCENT      15.0f
#define BRIGHTNESS_HOLD_VERY_SLOW_ZONE_PERCENT 5.0f
#define BRIGHTNESS_HOLD_MIN_REPEAT_MS_SLOW     15
#define BRIGHTNESS_HOLD_MIN_REPEAT_MS_VERY_SLOW 25
// Extra fine control below MIN_BRIGHTNESS_PERCENT (deep dim range).
#define BRIGHTNESS_HOLD_MIN_REPEAT_MS_DEEP_DIM 150

// Similar easing for temperature holds near the ends, so the very last
// warm/cool mix step doesn't feel like a "snap" to pure warm/cool.
#define TEMP_HOLD_SLOW_ZONE_RATIO              0.10f
#define TEMP_HOLD_VERY_SLOW_ZONE_RATIO         0.03f
#define TEMP_HOLD_MIN_REPEAT_MS_SLOW           30
#define TEMP_HOLD_MIN_REPEAT_MS_VERY_SLOW      60
 
static inline float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static inline float smoothstep01(float t)
{
    t = clampf(t, 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}

static inline float lerpf(float a, float b, float t)
{
    return a + (b - a) * t;
}

static inline float base_repeat_ms_for_hold(uint32_t held_ms)
{
    // Smooth acceleration to avoid a noticeable "speed jump" while holding.
    // Roughly matches the old 25ms -> 15ms -> 5ms steps, but transitions
    // gradually instead of abruptly.
    float ms = 25.0f;

    // 25ms -> 15ms over ~200ms (starting near 400ms held time).
    float t1 = smoothstep01(((float)held_ms - 400.0f) / 200.0f);
    ms = lerpf(25.0f, 15.0f, t1);

    // 15ms -> 5ms over ~200ms (starting near 1200ms held time).
    float t2 = smoothstep01(((float)held_ms - 1200.0f) / 200.0f);
    ms = lerpf(ms, 5.0f, t2);

    return ms;
}

static inline float brightness_end_min_repeat_ms(float dist_to_limit_percent)
{
    // Fade the end-stop easing in/out smoothly to avoid discrete zone edges.
    float dist = dist_to_limit_percent;
    if (dist < 0.0f) dist = 0.0f;

    float zone = BRIGHTNESS_HOLD_SLOW_ZONE_PERCENT; // 15%
    if (zone <= 0.0f) {
        return 0.0f;
    }

    float very_slow_ms = (float)BRIGHTNESS_HOLD_MIN_REPEAT_MS_VERY_SLOW;
    float slow_ms      = (float)BRIGHTNESS_HOLD_MIN_REPEAT_MS_SLOW;

    // 0..zone: very_slow_ms -> slow_ms
    // zone..2*zone: slow_ms -> 0
    if (dist >= 2.0f * zone) {
        return 0.0f;
    }
    if (dist <= zone) {
        float t = smoothstep01(dist / zone);
        return lerpf(very_slow_ms, slow_ms, t);
    }

    float t = smoothstep01((dist - zone) / zone);
    return lerpf(slow_ms, 0.0f, t);
}

static inline float brightness_deep_dim_min_repeat_ms(float b_percent)
{
    // Below MIN_BRIGHTNESS_PERCENT, we want extra fine control, but we also want
    // the speed transition to feel smooth as we cross the 5% boundary.
    float b = b_percent;
    if (b <= 0.0f) {
        return (float)BRIGHTNESS_HOLD_MIN_REPEAT_MS_DEEP_DIM;
    }

    // 0..MIN: deep_dim_ms -> very_slow_ms
    float min_b = MIN_BRIGHTNESS_PERCENT;
    if (min_b <= 0.0f) {
        return 0.0f;
    }

    float deep_dim_ms  = (float)BRIGHTNESS_HOLD_MIN_REPEAT_MS_DEEP_DIM;
    float very_slow_ms = (float)BRIGHTNESS_HOLD_MIN_REPEAT_MS_VERY_SLOW;

    if (b < min_b) {
        float t = smoothstep01(b / min_b);
        return lerpf(deep_dim_ms, very_slow_ms, t);
    }

    // Blend out the extra deep-dim slowdown over the next few percent above MIN.
    float blend_end = min_b + BRIGHTNESS_HOLD_VERY_SLOW_ZONE_PERCENT; // 5% -> 10%
    if (b < blend_end) {
        float t = smoothstep01((b - min_b) / (blend_end - min_b));
        return lerpf(very_slow_ms, 0.0f, t);
    }

    return 0.0f;
}

// --- Tap/double-tap tracking ---
static uint32_t tap_last_ms[PAD_COUNT] = {0};

static uint32_t button_press_time[PAD_COUNT] = {0};
static uint32_t last_repeat_time[PAD_COUNT] = {0};
static bool button_being_held[PAD_COUNT] = {0};
static bool dim_down_hold_clamp_at_min = false;


// --- Touch event processing task ---
static void touch_task(void *arg)
{
    static bool prev_pressed[PAD_COUNT] = {0};
    bool pressed_now[PAD_COUNT] = {0};
    static uint32_t power_press_ms = 0;
    static bool power_arm_neopixel = false;

    while (1) {
        uint32_t pad_intr = 0;
        uint32_t now = esp_timer_get_time() / 1000; // Current time in ms
        uint32_t raw_now[PAD_COUNT] = {0};

        // Poll raw values and generate edge events (rising edges) for each pad.
        for (int i = 0; i < PAD_COUNT; ++i) {
            touch_pad_read_raw_data(touch_pads[i], &raw_now[i]);
            pressed_now[i] = (raw_now[i] > threshold[i]);

            // Rising edge -> treat as a new tap/press event.
            if (pressed_now[i] && !prev_pressed[i]) {
                pad_intr |= (1UL << touch_pads[i]);
            }
        }

#if CONFIG_OTL_SERIAL_OUTPUT && CONFIG_OTL_LOG_TOUCH_RAW
        static uint32_t last_touch_raw_log_ms = 0;
        if ((now - last_touch_raw_log_ms) >= (uint32_t)CONFIG_OTL_TOUCH_RAW_LOG_INTERVAL_MS) {
            last_touch_raw_log_ms = now;
            OTL_LOGI_PERIODIC("touch_raw",
                              "PWR=%lu/%lu B+=%lu/%lu B-=%lu/%lu T+=%lu/%lu T-=%lu/%lu",
                              (unsigned long)raw_now[0], (unsigned long)threshold[0],
                              (unsigned long)raw_now[1], (unsigned long)threshold[1],
                              (unsigned long)raw_now[2], (unsigned long)threshold[2],
                              (unsigned long)raw_now[3], (unsigned long)threshold[3],
                              (unsigned long)raw_now[4], (unsigned long)threshold[4]);
        }
#endif

        // --- LED ON/OFF + optional NeoPixel enable on long-hold ---
        // NeoPixel is always off by default. To enable it, turn the light on
        // from OFF and keep holding the power pad for POWER_HOLD_NEOPIXEL_MS.
        bool power_pressed = pressed_now[0];
        bool power_pressed_edge = power_pressed && !prev_pressed[0];

        if (power_pressed_edge) {
            otl_public_state_t state = {0};
            power_press_ms = now;
            otl_state_get_public(&state);

            if (!state.is_on) {
                neopixel_enabled = false;
                power_arm_neopixel = true;
                otl_state_apply_light_update(&(otl_light_update_t) {
                    .set_power = true,
                    .power_on = true,
                }, OTL_CHANGE_SOURCE_TOUCH);
                OTL_LOG_TOUCHI("LED turned: ON");
            } else {
                neopixel_enabled = false;
                power_arm_neopixel = false;
                otl_state_apply_light_update(&(otl_light_update_t) {
                    .set_power = true,
                    .power_on = false,
                }, OTL_CHANGE_SOURCE_TOUCH);
                OTL_LOG_TOUCHI("LED turned: OFF");
            }
        }

        if (power_pressed && power_arm_neopixel && !neopixel_enabled &&
            (now - power_press_ms) >= POWER_HOLD_NEOPIXEL_MS) {
            neopixel_enabled = true;
            otl_state_lock();
            update_outputs();
            otl_state_unlock();
            OTL_LOG_TOUCHI("NeoPixel enabled");
        }

        if (!power_pressed) {
            power_arm_neopixel = false;
        }

        // --- Brightness UP ---
        if (pad_intr & (1UL << touch_pads[1])) {
            otl_public_state_t state = {0};
            otl_state_get_public(&state);

            // First press or double-tap
            if (!button_being_held[1]) {
                button_being_held[1] = true;
                button_press_time[1] = now;
                last_repeat_time[1] = now;

                if (now - tap_last_ms[1] < DOUBLE_TAP_MS) {
                    otl_state_apply_light_update(&(otl_light_update_t) {
                        .set_power = true,
                        .power_on = true,
                        .set_brightness = true,
                        .force_on_with_brightness = true,
                        .brightness_percent = (float)MAX_BRIGHTNESS_PERCENT,
                    }, OTL_CHANGE_SOURCE_TOUCH);
                    OTL_LOG_TOUCHI("Brightness set to MAX (%d%%)", MAX_BRIGHTNESS_PERCENT);
                    tap_last_ms[1] = now;
                } else {
                    float new_brightness = fminf(state.brightness_percent + (float)BRIGHT_STEP,
                                                 (float)MAX_BRIGHTNESS_PERCENT);
                    if (new_brightness != state.brightness_percent) {
                        otl_state_apply_light_update(&(otl_light_update_t) {
                            .set_power = true,
                            .power_on = true,
                            .set_brightness = true,
                            .force_on_with_brightness = true,
                            .brightness_percent = new_brightness,
                        }, OTL_CHANGE_SOURCE_TOUCH);
                        OTL_LOG_TOUCHI("Brightness increased to: %d%%", (int)lroundf(new_brightness));
                    }
                    tap_last_ms[1] = now;
                }
            }
        }

        // --- Brightness DOWN ---
        if (pad_intr & (1UL << touch_pads[2])) {
            otl_public_state_t state = {0};
            // If the light is currently OFF, start from the normal minimum so a
            // dim-down press never "flashes" on at a previously-saved high
            // brightness.
            otl_state_get_public(&state);
            bool was_off = !state.is_on;
            float current_brightness = was_off ? MIN_BRIGHTNESS_PERCENT : state.brightness_percent;
             
            if (!button_being_held[2]) {
                button_being_held[2] = true;
                button_press_time[2] = now;
                last_repeat_time[2] = now;
                // Prevent accidental entry into the deep-dim range (below the
                // double-tap minimum) on the same press-and-hold. If we started
                // above MIN, or we just turned on from OFF, clamp this hold at
                // MIN until the user releases and holds again.
                dim_down_hold_clamp_at_min = was_off || (current_brightness > MIN_BRIGHTNESS_PERCENT);

                if ((current_brightness >= MIN_BRIGHTNESS_PERCENT) && (now - tap_last_ms[2] < DOUBLE_TAP_MS)) {
                    otl_state_apply_light_update(&(otl_light_update_t) {
                        .set_power = true,
                        .power_on = true,
                        .set_brightness = true,
                        .force_on_with_brightness = true,
                        .brightness_percent = MIN_BRIGHTNESS_PERCENT,
                    }, OTL_CHANGE_SOURCE_TOUCH);
                    OTL_LOG_TOUCHI("Brightness set to MIN (%d%%)", (int)lroundf(MIN_BRIGHTNESS_PERCENT));
                    tap_last_ms[2] = now;
                } else {
                    float min_limit = (current_brightness > MIN_BRIGHTNESS_PERCENT) ? MIN_BRIGHTNESS_PERCENT : HOLD_MIN_BRIGHTNESS_PERCENT;
                    float new_brightness = fmaxf(current_brightness - (float)BRIGHT_STEP, min_limit);
                    if (new_brightness != current_brightness) {
                        otl_state_apply_light_update(&(otl_light_update_t) {
                            .set_power = true,
                            .power_on = true,
                            .set_brightness = true,
                            .force_on_with_brightness = true,
                            .brightness_percent = new_brightness,
                        }, OTL_CHANGE_SOURCE_TOUCH);
                        OTL_LOG_TOUCHI("Brightness decreased to: %d%%", (int)lroundf(new_brightness));
                    }
                    tap_last_ms[2] = now;
                }
            }
        }

        // --- Temp UP (cooler) ---
        if (pad_intr & (1UL << touch_pads[3])) {
            otl_public_state_t state = {0};
            otl_state_get_public(&state);
            if (state.is_on && !button_being_held[3]) {
                button_being_held[3] = true;
                button_press_time[3] = now;
                last_repeat_time[3] = now;
                 
                if (now - tap_last_ms[3] < DOUBLE_TAP_MS) {
                    otl_state_apply_light_update(&(otl_light_update_t) {
                        .set_temp_ratio = true,
                        .temp_ratio = 1.0f,
                    }, OTL_CHANGE_SOURCE_TOUCH);
                    OTL_LOG_TOUCHI("Temperature set to MAX COOL (1.0)");
                } else {
                    float new_temp = clampf(state.temp_ratio + TEMP_STEP, 0.0f, 1.0f);
                    if (new_temp != state.temp_ratio) {
                        otl_state_apply_light_update(&(otl_light_update_t) {
                            .set_temp_ratio = true,
                            .temp_ratio = new_temp,
                        }, OTL_CHANGE_SOURCE_TOUCH);
                        OTL_LOG_TOUCHI("White temp increase: %.2f | Warm temp decrease: %.2f", 
                                new_temp, 1.0f - new_temp);
                    }
                }
                tap_last_ms[3] = now;
            }
        }

        // --- Temp DOWN (warmer) ---
        if (pad_intr & (1UL << touch_pads[4])) {
            otl_public_state_t state = {0};
            otl_state_get_public(&state);
            if (state.is_on && !button_being_held[4]) {
                button_being_held[4] = true;
                button_press_time[4] = now;
                last_repeat_time[4] = now;
                 
                if (now - tap_last_ms[4] < DOUBLE_TAP_MS) {
                    otl_state_apply_light_update(&(otl_light_update_t) {
                        .set_temp_ratio = true,
                        .temp_ratio = 0.0f,
                    }, OTL_CHANGE_SOURCE_TOUCH);
                    OTL_LOG_TOUCHI("Temperature set to MAX WARM (0.0)");
                } else {
                    float new_temp = clampf(state.temp_ratio - TEMP_STEP, 0.0f, 1.0f);
                    if (new_temp != state.temp_ratio) {
                        otl_state_apply_light_update(&(otl_light_update_t) {
                            .set_temp_ratio = true,
                            .temp_ratio = new_temp,
                        }, OTL_CHANGE_SOURCE_TOUCH);
                        OTL_LOG_TOUCHI("Cool temp decrease: %.2f | Warm temp increase: %.2f", 
                                new_temp, 1.0f - new_temp);
                    }
                }
                tap_last_ms[4] = now;
            }
        }
        // Handle button hold and repeat - this is independent of queue events
        // to ensure consistent timing
        now = esp_timer_get_time() / 1000; // Update current time
        bool want_hold_stepper = false;
         
        // Check each button for hold state
        for (int i = 1; i < PAD_COUNT; i++) {
            if (button_being_held[i]) {
                uint32_t held_ms = now - button_press_time[i];
                float base_repeat_ms = base_repeat_ms_for_hold(held_ms);

                // Check if button is still pressed; if released, stop repeating
                if (!pressed_now[i]) {
                    button_being_held[i] = false;
                    if (i == 2) {
                        dim_down_hold_clamp_at_min = false;
                    }
                    continue;
                }

                // During the initial hold delay, keep last_repeat_time fresh so
                // we don't get a big jump when the hold action starts.
                if (held_ms <= HOLD_DELAY_MS) {
                    last_repeat_time[i] = now;
                    continue;
                }

                // Perform the appropriate hold action
                switch (i) {
                    case 1: // Brightness UP (continuous, high resolution)
                    case 2: // Brightness DOWN (continuous, high resolution)
                    {
                        want_hold_stepper = true;
                        uint32_t dt_ms = now - last_repeat_time[i];
                        last_repeat_time[i] = now;
                        otl_public_state_t state = {0};
                        otl_state_get_public(&state);

                        float effective_repeat_ms = base_repeat_ms;
                        float dist_to_limit = 0.0f;
                        float min_brightness_limit = MIN_BRIGHTNESS_PERCENT;
                        if (i == 2) {
                            min_brightness_limit = dim_down_hold_clamp_at_min ? MIN_BRIGHTNESS_PERCENT : HOLD_MIN_BRIGHTNESS_PERCENT;
                        }
                        if (i == 1) {
                            dist_to_limit = (float)MAX_BRIGHTNESS_PERCENT - state.brightness_percent;
                        } else {
                            dist_to_limit = state.brightness_percent - min_brightness_limit;
                            if (dist_to_limit < 0.0f) {
                                dist_to_limit = 0.0f;
                            }
                        }
                        float end_min_ms = brightness_end_min_repeat_ms(dist_to_limit);
                        if (end_min_ms > effective_repeat_ms) {
                            effective_repeat_ms = end_min_ms;
                        }

                        float deep_min_ms = brightness_deep_dim_min_repeat_ms(state.brightness_percent);
                        if (deep_min_ms > effective_repeat_ms) {
                            effective_repeat_ms = deep_min_ms;
                        }

                        float rate_percent_per_s = ((float)BRIGHT_STEP * 1000.0f) / effective_repeat_ms;
                        float delta_percent = rate_percent_per_s * ((float)dt_ms / 1000.0f);
                        if (delta_percent <= 0.0f) {
                            break;
                        }

                        float new_brightness = state.brightness_percent;
                        if (i == 1) {
                            new_brightness = fminf(state.brightness_percent + delta_percent, (float)MAX_BRIGHTNESS_PERCENT);
                        } else {
                            new_brightness = fmaxf(state.brightness_percent - delta_percent, min_brightness_limit);
                        }

                        if (new_brightness != state.brightness_percent) {
                            otl_state_apply_light_update(&(otl_light_update_t) {
                                .set_power = true,
                                .power_on = true,
                                .set_brightness = true,
                                .force_on_with_brightness = true,
                                .brightness_percent = new_brightness,
                            }, OTL_CHANGE_SOURCE_TOUCH);
                        }
                        break;
                    }

                    case 3: // Temp UP
                    case 4: // Temp DOWN
                    {
                        otl_public_state_t state = {0};
                        otl_state_get_public(&state);
                        if (!state.is_on) {
                            break;
                        }

                        uint32_t dt_ms = now - last_repeat_time[i];
                        last_repeat_time[i] = now;

                        // Don't let the effective repeat be faster than our loop cadence.
                        float effective_repeat_ms = base_repeat_ms;
                        if (effective_repeat_ms < (float)dt_ms) {
                            effective_repeat_ms = (float)dt_ms;
                        }

                        float dist_to_limit = (i == 3) ? (1.0f - state.temp_ratio) : state.temp_ratio;
                        if (dist_to_limit < TEMP_HOLD_VERY_SLOW_ZONE_RATIO) {
                            if (effective_repeat_ms < TEMP_HOLD_MIN_REPEAT_MS_VERY_SLOW) {
                                effective_repeat_ms = (float)TEMP_HOLD_MIN_REPEAT_MS_VERY_SLOW;
                            }
                        } else if (dist_to_limit < TEMP_HOLD_SLOW_ZONE_RATIO) {
                            if (effective_repeat_ms < TEMP_HOLD_MIN_REPEAT_MS_SLOW) {
                                effective_repeat_ms = (float)TEMP_HOLD_MIN_REPEAT_MS_SLOW;
                            }
                        }

                        float rate_ratio_per_s = (TEMP_STEP * 1000.0f) / effective_repeat_ms;
                        float delta_ratio = rate_ratio_per_s * ((float)dt_ms / 1000.0f);
                        if (delta_ratio <= 0.0f) {
                            break;
                        }

                        float new_temp = clampf(state.temp_ratio + ((i == 3) ? delta_ratio : -delta_ratio), 0.0f, 1.0f);
                        if (new_temp != state.temp_ratio) {
                            otl_state_apply_light_update(&(otl_light_update_t) {
                                .set_temp_ratio = true,
                                .temp_ratio = new_temp,
                            }, OTL_CHANGE_SOURCE_TOUCH);
                        }
                        break;
                    }
                }
            }
        }
        pwm_hold_stepper_set_enabled(want_hold_stepper);

        // Remember current pressed state for edge detection next loop
        for (int i = 0; i < PAD_COUNT; ++i) {
            prev_pressed[i] = pressed_now[i];
        }

        // Small delay to keep CPU usage reasonable.
        // Note: if the FreeRTOS tick is 100 Hz (10 ms), pdMS_TO_TICKS(5) == 0,
        // which would cause this task to run almost continuously and starve
        // the idle task, triggering the task watchdog.
        TickType_t delay_ticks = pdMS_TO_TICKS(TOUCH_LOOP_DELAY_MS);
        if (delay_ticks == 0) {
            delay_ticks = 1;
        }
        vTaskDelay(delay_ticks);
    }
}

// ---------------------------- Radar (LD2410B) task ----------------------------
#if CONFIG_OTL_PRESENCE_SENSOR
static void radar_task(void *arg)
{
    otl_public_state_t state = {0};
    // Configure initial state from pin level + last_sample on both candidate OUT pins
    int level15 = gpio_get_level(RADAR_OUT_GPIO);
    int level14 = gpio_get_level(RADAR_OUT_GPIO_ALT);
    bool out_state = (level14 == 1) || (level15 == 1);
    bool moving = radar_last_sample.valid && (radar_last_sample.targetType & 0x01);
    bool stationary = radar_last_sample.valid && (radar_last_sample.targetType & 0x02);
    bool initial_presence = out_state || moving || stationary;
    otl_state_lock();
    radar_occupied = initial_presence;
    otl_state_unlock();
    otl_state_set_presence(initial_presence, OTL_CHANGE_SOURCE_OCCUPANCY);

    uint32_t last_log_ms      = 0;
    uint32_t last_presence_ms = esp_timer_get_time() / 1000;
    uint32_t presence_candidate_since_ms = 0;
    bool     presence_candidate_active   = false;

    while (1) {
        bool currently_occupied = false;
        // 1) Read any pending UART data and update radar_last_sample
        uint8_t buf[64];
        int len = uart_read_bytes(RADAR_UART_NUM, buf, sizeof(buf), 0);
        if (len > 0) {
            if ((size_t)len > sizeof(radar_rx_buf) - radar_rx_len) {
                len = (int)(sizeof(radar_rx_buf) - radar_rx_len);
            }
            if (len > 0) {
                memcpy(&radar_rx_buf[radar_rx_len], buf, len);
                radar_rx_len += len;
                radar_parse_frames();
            }
        }

        // 2) Evaluate presence based on OUT pins + moving/static flags within range
        int out15_level = gpio_get_level(RADAR_OUT_GPIO);
        int out14_level = gpio_get_level(RADAR_OUT_GPIO_ALT);
        out_state = (out14_level == 1) || (out15_level == 1);

        // Consider moving targets as "near" only within RADAR_MOVING_NEAR_MAX_CM,
        // and stationary targets as "near" within RADAR_PRESENCE_MAX_DIST_CM.
        // The light will only turn off once BOTH movDist and statDist are
        // beyond these ranges (or no valid distances are reported), and this
        // condition has held for RADAR_ABSENCE_TIMEOUT_MS.
        moving = radar_last_sample.valid &&
                 (radar_last_sample.movingDist > 0) &&
                 (radar_last_sample.movingDist <= RADAR_MOVING_NEAR_MAX_CM);
        stationary = radar_last_sample.valid &&
                     (radar_last_sample.staticDist > 0) &&
                     (radar_last_sample.staticDist <= RADAR_PRESENCE_MAX_DIST_CM);

        // Presence is based solely on distances (movDist/statDist) within the
        // configured range. We do NOT use the OUT pin for dimming, since it
        // can stay high even when the person has left the near field.
        bool new_presence = moving || stationary;

        uint32_t now_ms = esp_timer_get_time() / 1000;

        // Update last_presence_ms whenever we see presence, and track how long
        // continuous presence has been observed for "turn-on" hysteresis.
        if (new_presence) {
            last_presence_ms = now_ms;
            if (!presence_candidate_active) {
                presence_candidate_active = true;
                presence_candidate_since_ms = now_ms;
            }
        } else {
            presence_candidate_active = false;
        }

        // Log periodically so we can see what the radar is doing,
        // even if presence does not change.
        if (now_ms - last_log_ms > 2000) {
            otl_state_get_public(&state);
            OTL_LOG_RADARI("presence=%s OUT14=%d OUT15=%d moving=%d stationary=%d movDist=%ucm statDist=%ucm",
                           (state.presence ? "ON" : "OFF"),
                           out14_level, out15_level,
                           moving ? 1 : 0, stationary ? 1 : 0,
                           (unsigned)radar_last_sample.movingDist,
                           (unsigned)radar_last_sample.staticDist);
            last_log_ms = now_ms;
        }

        // Apply hysteresis:
        //  - turn ON only after presence has been continuous for
        //    RADAR_PRESENCE_ON_DELAY_MS
        //  - turn OFF only after absence has been continuous for
        //    RADAR_ABSENCE_TIMEOUT_MS
        otl_state_get_public(&state);
        otl_state_lock();
        currently_occupied = radar_occupied;
        otl_state_unlock();

        if (new_presence && !currently_occupied &&
            presence_candidate_active &&
            (now_ms - presence_candidate_since_ms) > RADAR_PRESENCE_ON_DELAY_MS) {
            otl_state_set_presence(true, OTL_CHANGE_SOURCE_OCCUPANCY);
            handle_occupancy_gained();
        } else if (!new_presence && currently_occupied &&
                   (now_ms - last_presence_ms) > RADAR_ABSENCE_TIMEOUT_MS) {
            otl_state_set_presence(false, OTL_CHANGE_SOURCE_OCCUPANCY);
            handle_occupancy_lost();
        }

        TickType_t delay_ticks = pdMS_TO_TICKS(RADAR_TASK_LOOP_INTERVAL_MS);
        if (delay_ticks == 0) {
            delay_ticks = 1;
        }
        vTaskDelay(delay_ticks);
    }
}
#endif

// ---------------------------- Sensor task (ALS + NTC) -------------------------
static void sensor_task(void *arg)
{
    (void)arg;
    TickType_t last_wake = xTaskGetTickCount();
#if CONFIG_OTL_SERIAL_OUTPUT && (CONFIG_OTL_LOG_STATUS || CONFIG_OTL_SENSOR_DEBUG)
    uint32_t log_elapsed_ms = SENSOR_READ_INTERVAL_MS;
#endif

    while (1) {
        float ntc_temp  = read_ntc_temperature();
        float chip_temp = read_chip_temperature();

        thermal_update(ntc_temp, chip_temp);

#if CONFIG_OTL_SERIAL_OUTPUT && (CONFIG_OTL_LOG_STATUS || CONFIG_OTL_SENSOR_DEBUG)
        log_elapsed_ms += THERMAL_POLL_INTERVAL_MS;
        if (log_elapsed_ms >= SENSOR_READ_INTERVAL_MS) {
            otl_public_state_t state = {0};
            log_elapsed_ms = 0;
            float lux = read_als_lux();
            otl_state_get_public(&state);
#if CONFIG_OTL_LOG_STATUS
            const char *light_state = state.is_on ? "ON" : "OFF";
            float temp_cool_ratio = state.temp_ratio;
            float temp_warm_ratio = 1.0f - state.temp_ratio;
            if (!isnan(chip_temp)) {
                OTL_LOG_STATUSI("Light %s | B %.0f%% | Temp %.2f (cool %.0f%%/warm %.0f%%) | Ambient %.0f lux | NTC %.1f °C | Chip %.1f °C",
                                light_state,
                                state.brightness_percent,
                                temp_cool_ratio,
                                temp_cool_ratio * 100.0f,
                                temp_warm_ratio * 100.0f,
                                lux,
                                ntc_temp,
                                chip_temp);
            } else {
                OTL_LOG_STATUSI("Light %s | B %.0f%% | Temp %.2f (cool %.0f%%/warm %.0f%%) | Ambient %.0f lux | NTC %.1f °C",
                                light_state,
                                state.brightness_percent,
                                temp_cool_ratio,
                                temp_cool_ratio * 100.0f,
                                temp_warm_ratio * 100.0f,
                                lux,
                                ntc_temp);
            }
#else
            (void)lux;
#endif
        }
#endif

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(THERMAL_POLL_INTERVAL_MS));
    }
}

// ---------------------------- app_main ----------------------------------------

void app_main(void)
{
#if !CONFIG_OTL_SERIAL_OUTPUT
    esp_log_level_set("*", ESP_LOG_NONE);
#endif

    // 1. LEDC PWM
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer_cfg));
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_ch_cool));
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_ch_warm));
    ESP_ERROR_CHECK(ledc_fade_func_install(0));

    const esp_timer_create_args_t pwm_hold_step_timer_args = {
        .callback = &pwm_hold_stepper_timer_cb,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "pwm_hold_stepper",
        .skip_unhandled_events = true,
    };
    ESP_ERROR_CHECK(esp_timer_create(&pwm_hold_step_timer_args, &pwm_hold_step_timer));
#if !CONFIG_OTL_NONOVERLAP_PWM
    const esp_timer_create_args_t pwm_off_timer_cool_args = {
        .callback = &pwm_off_timer_cb,
        .arg = &pwm_off_args_cool,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "pwm_off_cool",
        .skip_unhandled_events = true,
    };
    ESP_ERROR_CHECK(esp_timer_create(&pwm_off_timer_cool_args, &pwm_off_timer_cool));

    const esp_timer_create_args_t pwm_off_timer_warm_args = {
        .callback = &pwm_off_timer_cb,
        .arg = &pwm_off_args_warm,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "pwm_off_warm",
        .skip_unhandled_events = true,
    };
    ESP_ERROR_CHECK(esp_timer_create(&pwm_off_timer_warm_args, &pwm_off_timer_warm));
#endif
#if CONFIG_OTL_NONOVERLAP_PWM
    pwm_nonoverlap_mutex = xSemaphoreCreateMutex();
    if (pwm_nonoverlap_mutex == NULL) {
        abort();
    }
    const esp_timer_create_args_t pwm_timer_args = {
        .callback = &pwm_nonoverlap_timer_cb,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "pwm_nonoverlap",
        .skip_unhandled_events = true,
    };
    ESP_ERROR_CHECK(esp_timer_create(&pwm_timer_args, &pwm_nonoverlap_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(pwm_nonoverlap_timer, PWM_SOFT_FADE_UPDATE_US));
#endif

#if CONFIG_OTL_SERIAL_OUTPUT && CONFIG_OTL_LOG_PWM_DUTY
    const esp_timer_create_args_t pwm_log_timer_args = {
        .callback = &pwm_log_timer_cb,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "pwm_log",
        .skip_unhandled_events = true,
    };
    ESP_ERROR_CHECK(esp_timer_create(&pwm_log_timer_args, &pwm_log_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(pwm_log_timer, (uint64_t)CONFIG_OTL_PWM_LOG_INTERVAL_MS * 1000ULL));
#endif

    otl_state_mutex = xSemaphoreCreateMutex();
    if (otl_state_mutex == NULL) {
        abort();
    }

    // 2. WS2812 via RMT LED-Strip driver
    led_strip_config_t strip_config = {
        .strip_gpio_num   = WS2812_GPIO,
        .max_leds         = WS2812_NUM_PIXELS,
        .led_model        = LED_MODEL_WS2812,
        .flags.invert_out = false,
    };
    led_strip_rmt_config_t rmt_cfg = {
        .clk_src   = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000, // 10 MHz
        .mem_block_symbols = 64,
        .flags.with_dma = false,
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_cfg, &strip));
    led_strip_clear(strip);

    // 3. Radar OUT pin (LD2410B occupancy) + UART for moving/stationary
#if CONFIG_OTL_PRESENCE_SENSOR
    gpio_config_t radar_cfg = {
        .pin_bit_mask = (1ULL << RADAR_OUT_GPIO) | (1ULL << RADAR_OUT_GPIO_ALT),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&radar_cfg));

    // Radar UART (LD2410B) configuration
    uart_config_t uart_cfg = {
        .baud_rate  = RADAR_UART_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(RADAR_UART_NUM, 1024, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(RADAR_UART_NUM, &uart_cfg));
    ESP_ERROR_CHECK(uart_set_pin(RADAR_UART_NUM,
                                 RADAR_TX_GPIO,  // TX -> radar RX
                                 RADAR_RX_GPIO,  // RX <- radar TX
                                 UART_PIN_NO_CHANGE,
                                 UART_PIN_NO_CHANGE));
#endif

    // 4. Touch pads - dynamic baseline / threshold (value rises on touch)
    ESP_ERROR_CHECK(touch_pad_init());

    for (int i = 0; i < PAD_COUNT; ++i) {
        ESP_ERROR_CHECK(touch_pad_config(touch_pads[i]));
        // Standard configuration; on this board raw value rises when touched
        ESP_ERROR_CHECK(touch_pad_set_cnt_mode(touch_pads[i],
                                               TOUCH_PAD_SLOPE_7,
                                               TOUCH_PAD_TIE_OPT_LOW));
    }

    // General touch sensing parameters
    ESP_ERROR_CHECK(touch_pad_set_charge_discharge_times(5));
    ESP_ERROR_CHECK(touch_pad_set_voltage(TOUCH_HVOLT_2V7, TOUCH_LVOLT_0V5, TOUCH_HVOLT_ATTEN_0V));
    // Shorter measurement interval for faster response (~30–40 ms between updates)
    ESP_ERROR_CHECK(touch_pad_set_measurement_interval(1000));

    // Filter configuration
    touch_filter_config_t filter_config = {
        .mode = TOUCH_PAD_FILTER_IIR_16,
        .debounce_cnt = 2,
        .noise_thr = 3,
        .jitter_step = 6,
        .smh_lvl = TOUCH_PAD_SMOOTH_IIR_4,
    };
    ESP_ERROR_CHECK(touch_pad_filter_set_config(&filter_config));
    ESP_ERROR_CHECK(touch_pad_filter_enable());
    
    // Start the touch FSM
    ESP_ERROR_CHECK(touch_pad_set_fsm_mode(TOUCH_FSM_MODE_TIMER));
    ESP_ERROR_CHECK(touch_pad_fsm_start());

    // Wait for measuring to complete (this can take longer after cold boot / when
    // power/ground conditions change, so don't assume it is instant).
    TickType_t t0 = xTaskGetTickCount();
    while (!touch_pad_meas_is_done() &&
           (xTaskGetTickCount() - t0) < pdMS_TO_TICKS(TOUCH_MEAS_WAIT_MS)) {
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    // Let the filter/FSM settle before sampling baselines.
    vTaskDelay(pdMS_TO_TICKS(TOUCH_SETTLE_MS));

    // Calibrate touch pads using Arduino-style baseline/thresholds.
    // Cold power-on can leave the touch front-end drifting for a short time,
    // so validate the first calibration and retry if needed.
    touch_calibrate_startup();

    // Using polling-based touch handling in touch_task() instead of ISR

    // 5. ADC (ALS + NTC)
    adc_init();

    // 5b. ESP32 internal temperature sensor
    chip_temp_init();
    thermal_brightness_cap_percent = (float)MAX_BRIGHTNESS_PERCENT;

#if CONFIG_OTL_WIFI_ENABLE
    esp_err_t wifi_err = otl_net_start_wifi();
    if (wifi_err != ESP_OK) {
        OTL_LOGE("wifi", "Start failed: %s", esp_err_to_name(wifi_err));
    }
#endif

#if CONFIG_OTL_CIRCADIAN_ENABLE
    esp_err_t circadian_err = otl_circadian_start(otl_circadian_apply_cb, NULL);
    if (circadian_err != ESP_OK) {
        OTL_LOGE("circadian", "Start failed: %s", esp_err_to_name(circadian_err));
    }
#endif

    // 6. Start tasks with adequate stack sizes
    // touch_task and radar_task need larger stacks due to complex state
    // sensor_task and recalib_task are simpler and can use smaller stacks
    xTaskCreate(touch_task, "touch_task", 4096, NULL, 5, NULL);
    xTaskCreate(sensor_task, "sensor_task", 2560, NULL, 5, NULL);
    xTaskCreate(recalibration_task, "recalib_task", 2560, NULL, 2, NULL);
#if CONFIG_OTL_PRESENCE_SENSOR
    xTaskCreate(radar_task, "radar_task", 4096, NULL, 4, NULL);
#endif

    // Initial output state
    otl_state_lock();
    update_outputs();
    otl_state_unlock();
    
    OTL_LOGI("main", "Open Task Light initialization complete");
}
