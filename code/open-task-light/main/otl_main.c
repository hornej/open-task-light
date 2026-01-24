// main.c – ESP‑IDF skeleton that mirrors your Arduino sketch
// Tested with ESP‑IDF v5.x on an ESP32‑S3 module
// -----------------------------------------------------------
// This is NOT a drop‑in replacement: it is a structured starting
// point that compiles under idf.py and retains the logic of your
// original sketch (PWM dimming, capacitive‑touch UI, WS2812 RGB
// status LED, ADC sensors).  Flesh out the TODOs as you verify
// hardware pin‑mapping and tweak thresholds.

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>     
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#if CONFIG_OTL_SERIAL_OUTPUT
#define OTL_LOGI(tag, fmt, ...) ESP_LOGI(tag, fmt, ##__VA_ARGS__)
#define OTL_LOGW(tag, fmt, ...) ESP_LOGW(tag, fmt, ##__VA_ARGS__)
#define OTL_LOGE(tag, fmt, ...) ESP_LOGE(tag, fmt, ##__VA_ARGS__)
#define OTL_LOGD(tag, fmt, ...) ESP_LOGD(tag, fmt, ##__VA_ARGS__)
#else
#define OTL_LOGI(tag, fmt, ...) do { (void)(tag); (void)(fmt); } while (0)
#define OTL_LOGW(tag, fmt, ...) do { (void)(tag); (void)(fmt); } while (0)
#define OTL_LOGE(tag, fmt, ...) do { (void)(tag); (void)(fmt); } while (0)
#define OTL_LOGD(tag, fmt, ...) do { (void)(tag); (void)(fmt); } while (0)
#endif
#include "esp_timer.h"
#include "driver/ledc.h"
#include "driver/touch_pad.h"
#include "esp_adc/adc_oneshot.h"
#include "driver/gpio.h"
#if CONFIG_OTL_PRESENCE_SENSOR
#include "driver/uart.h"
#endif
#include "led_strip.h" 
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "freertos/queue.h"
#include "esp_intr_alloc.h"

// Touch event queue removed - using polling-based approach in touch_task()



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
#define TOUCH_SETTLE_MS           100   // Delay for filter/FSM to settle before calibration
#define SENSOR_READ_INTERVAL_MS   10000 // Interval between ALS/NTC sensor readings
#define RECALIB_INTERVAL_MS       60000 // Interval between touch recalibration attempts
#define RADAR_MAX_ENERGY          100   // Maximum energy value from LD2410 radar

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

#define DIM_OUT_COOL_GPIO  36   // PWM output for cool white (verify that GPIO36 is routed to LEDC-capable pad on S3)
#define DIM_OUT_WARM_GPIO  35   // PWM output for warm white
#define WS2812_GPIO        40   // RMT / LED-Strip output pin

#define ALS_GPIO           5    // ALS‑PT19 analog input (ADC)
#define NTC_GPIO           16   // NTC thermistor divider analog input (ADC)

// ---------------------------- LEDC PWM ----------------------------
#define OTL_LEDC_DUTY_RES LEDC_TIMER_12_BIT
#define OTL_LEDC_FREQ_HZ  19531

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
static const uint32_t PWM_MAX_DUTY = (1U << OTL_LEDC_DUTY_RES) - 1U;

// ---------------------------- WS2812 (RMT LED-Strip) ----------------------------
 #define WS2812_NUM_PIXELS  1
 static led_strip_handle_t strip;
 static bool neopixel_enabled = false;

// ---------------------------- PWM fade smoothing -------------------------------
// Fade between duty targets in hardware so we don't "skip" duty steps.
// Used by the hold stepper (0->max in ~1.2s at 12-bit/95% cap).
#define PWM_HOLD_STEP_UPDATE_US       300
#define PWM_FADE_STEP                 1
// Try to keep fade duration roughly constant for small deltas so you don't get
// "step + settle" behavior at low brightness.
#define PWM_FADE_TARGET_TIME_MS       10
#define PWM_FADE_OFF_TIME_MS          200
#define PWM_FADE_MAX_CYCLES_PER_STEP  1000

#if !CONFIG_OTL_NONOVERLAP_PWM
static void pwm_set_duty_smooth_timed(ledc_mode_t speed_mode, ledc_channel_t channel, uint32_t target_duty, uint32_t target_time_ms)
{
    uint32_t current_duty = ledc_get_duty(speed_mode, channel);
    if (current_duty == target_duty) {
        return;
    }

    uint32_t delta = (current_duty > target_duty) ? (current_duty - target_duty) : (target_duty - current_duty);
    if (delta == 0) {
        return;
    }

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

    if ((warm_width + cool_width) > PWM_MAX_DUTY) {
        if (warm_width >= PWM_MAX_DUTY) {
            warm_width = PWM_MAX_DUTY;
            cool_width = 0;
        } else {
            cool_width = PWM_MAX_DUTY - warm_width;
        }
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
    if (total_cycles < (uint64_t)delta_max) {
        total_cycles = (uint64_t)delta_max;
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

static const int   BRIGHT_STEP      = 1;
static const float TEMP_STEP        = 0.015f;
static const int   DOUBLE_TAP_MS    = 300;
// Perceptual brightness gamma (lower = brighter/less aggressive low-end dimming)
static const float GAMMA_CORRECTION = 1.6f;
// Limit maximum brightness to avoid overheating
static const int   MAX_BRIGHTNESS_PERCENT = 95;
static const float MIN_BRIGHTNESS_PERCENT = 5.0f;
// While holding dim-down, allow a deeper range below the normal minimum.
// HOLD_MIN_BRIGHTNESS_PERCENT is the internal floor used for holds; the mapping
// below converts that to a physical PWM duty floor (HOLD_MIN_PWM_DUTY_RATIO).
static const float HOLD_MIN_BRIGHTNESS_PERCENT = 0.0f;
static const float HOLD_MIN_PWM_DUTY_RATIO     = 0.01f; // 1% total duty floor
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
            uint32_t target = pwm_hold_target_total_duty;
            pwm_hold_stepper_apply_total(target);
            pwm_hold_current_total_duty = target;
        }
    }
}

#if CONFIG_OTL_PRESENCE_SENSOR
// ---------------------------- Occupancy (LD2410B) ----------------------------
// The LD2410B's digital OUT pin drives this occupancy state.
// We use it to fade the light off when the room is empty, and
// fade back in to the previous state when occupancy returns.
static bool radar_occupied        = false;
static bool occupancy_faded_out   = false;
static bool saved_led_state       = false;
static int  saved_brightness      = 0;
static float saved_temp_ratio     = 0.5f;
// Presence as determined from radar UART + OUT pin
static bool radar_presence        = false;
// Maximum distance (in cm) for presence to count when using UART data.
// For moving targets we use a tighter radius (3.0 m), for stationary
// targets we allow a bit further.
static const uint16_t RADAR_MOVING_NEAR_MAX_CM   = 300;  // 3.0 m
static const uint16_t RADAR_PRESENCE_MAX_DIST_CM = 500;  // 5.0 m for stationary
// How long absence must persist before we dim/turn off (ms)
static const uint32_t RADAR_ABSENCE_TIMEOUT_MS   = 30000;
// How long continuous presence must persist before we treat the room as
// occupied again and turn the light back on (ms)
static const uint32_t RADAR_PRESENCE_ON_DELAY_MS = 3000;
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
static const float touch_threshold_margin_pct[PAD_COUNT] = { 0.05f, 0.03f, 0.03f, 0.03f, 0.03f };

static void calibrate_touch_pads(void)
{
    OTL_LOGI("touch", "Calibrating touch pads (dynamic baseline/threshold)...");

    // Strategy adapted from the Arduino sketch:
    //  - sample each pad several times with no touch
    //  - compute a baseline per pad
    //  - set a per-pad threshold as baseline * factor[i]
    //  - on this board, touch makes the value RISE,
    //    so we treat \"value ABOVE threshold\" as a touch
    for (int i = 0; i < PAD_COUNT; ++i) {
        uint64_t sum = 0;
        int valid_samples = 0;
        uint32_t min_val = 0xFFFFFFFFu;
        uint32_t max_val = 0;

        for (int s = 0; s < TOUCH_CAL_SAMPLES; ++s) {
            uint32_t val;
            touch_pad_read_raw_data(touch_pads[i], &val);
            // Some pads can briefly report 0 right after start-up.
            // Ignore those so we don't get a bogus baseline of 0.
            if (val != 0) {
                sum += val;
                valid_samples++;
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

        ESP_ERROR_CHECK(touch_pad_set_thresh(touch_pads[i], threshold[i]));

        OTL_LOGI("touch", "Pad %d baseline=%lu threshold=%lu noise_span=%lu margin=%lu",
                 i,
                 (unsigned long)baseline[i],
                 (unsigned long)threshold[i],
                 (unsigned long)noise_span,
                 (unsigned long)margin);
    }
}

static void recalibration_task(void *arg)
{
    while (1) {
        // Wait for a period with no touch activity
        vTaskDelay(pdMS_TO_TICKS(RECALIB_INTERVAL_MS));
        
        // Only recalibrate if no buttons are being touched
        bool can_recalibrate = true;
        for (int i = 0; i < PAD_COUNT; ++i) {
            uint32_t val;
            touch_pad_read_raw_data(touch_pads[i], &val);
            if (val > threshold[i]) {
                can_recalibrate = false;
                break;
            }
        }
        
        if (can_recalibrate) {
            OTL_LOGI("touch", "Performing periodic recalibration");
            calibrate_touch_pads();
        }
    }
}

// ---------------------------- Radar UART frame parsing ------------------------
#if CONFIG_OTL_PRESENCE_SENSOR
// Parse incoming LD2410 frames into radar_last_sample, similar to LD2410B.ino.
static void radar_parse_frames(void)
{
    bool any = false;
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
            any = true;
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

    (void)any; // currently unused, but kept for future logging if needed
}
#endif

// Touch ISR removed - using polling-based approach in touch_task() for simplicity
// and to avoid slow peripheral reads in interrupt context

// ---------------------------- Helper functions ---------------------------------
static void ws2812_set_rgb(uint8_t r, uint8_t g, uint8_t b)
{
    led_strip_set_pixel(strip, 0, r, g, b);
    led_strip_refresh(strip);
}

// Forward declaration so occupancy helpers can call it.
static void update_outputs(void);

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
            brightness_percent = b;
            update_outputs();
            vTaskDelay(pdMS_TO_TICKS(delay_ms));
        }
    } else {
        for (int b = start; b >= end; b -= step) {
            brightness_percent = b;
            update_outputs();
            vTaskDelay(pdMS_TO_TICKS(delay_ms));
        }
    }
}

static void handle_occupancy_lost(void)
{
    radar_occupied = false;

    // Only fade out if the light is currently on and
    // we haven't already faded it out.
    if (!led_state || occupancy_faded_out) {
        return;
    }

    saved_led_state  = led_state;
    saved_brightness = (int)lroundf(fminf(brightness_percent, (float)MAX_BRIGHTNESS_PERCENT));
    saved_temp_ratio = temp_ratio;

    // Fade down to 0% brightness, then turn off.
    fade_brightness((int)lroundf(brightness_percent), 0, BRIGHT_STEP, 15);
    led_state = false;
    update_outputs();

    occupancy_faded_out = true;
}

static void handle_occupancy_gained(void)
{
    radar_occupied = true;

    // Only fade back in if we previously faded out the light
    // and the saved state indicates it should be on.
    if (!occupancy_faded_out || !saved_led_state) {
        return;
    }

    led_state  = true;
    temp_ratio = saved_temp_ratio;

    // Fade from 0 up to the saved brightness.
    brightness_percent = 0;
    update_outputs();
    fade_brightness(0, saved_brightness, BRIGHT_STEP, 15);

    occupancy_faded_out = false;
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

    // Apply gamma correction so that brightness_percent feels more linear
    // to human perception for both PWM and WS2812 output.
    //
    // Shift the physical mapping so that 1% brightness produces roughly
    // the same PWM duty as the old 10% level, while still allowing 2%,
    // 3%, ... to be progressively brighter. Also clamp the physical max
    // to MAX_BRIGHTNESS_PERCENT to avoid overheating.
    float b = brightness_percent;
    if (b < HOLD_MIN_BRIGHTNESS_PERCENT) b = HOLD_MIN_BRIGHTNESS_PERCENT;
    if (b > (float)MAX_BRIGHTNESS_PERCENT) b = (float)MAX_BRIGHTNESS_PERCENT;

    float gamma_scaled = 0.0f;
    if (b < MIN_BRIGHTNESS_PERCENT) {
        float linear_min = (MIN_BRIGHTNESS_PERCENT + (float)MIN_BRIGHTNESS_OFFSET_PERCENT) / 100.0f;
        if (linear_min < 0.0f) linear_min = 0.0f;
        if (linear_min > 1.0f) linear_min = 1.0f;
        float duty_at_min = powf(linear_min, GAMMA_CORRECTION);

        float t = b / MIN_BRIGHTNESS_PERCENT; // 0..1
        if (t < 0.0f) t = 0.0f;
        if (t > 1.0f) t = 1.0f;
        gamma_scaled = HOLD_MIN_PWM_DUTY_RATIO + t * (duty_at_min - HOLD_MIN_PWM_DUTY_RATIO);
    } else {
        float linear = (b + (float)MIN_BRIGHTNESS_OFFSET_PERCENT) / 100.0f;  // 1% -> ~0.15, 2% -> ~0.16, ...
        if (linear < 0.0f) linear = 0.0f;
        if (linear > 1.0f) linear = 1.0f;
        gamma_scaled = powf(linear, GAMMA_CORRECTION);
    }

    // Hard cap on physical PWM duty to limit maximum output/heat.
    // Note: MAX_BRIGHTNESS_PERCENT is a user-facing "%", but after gamma/offset
    // it may otherwise exceed that duty ratio unless we clamp here.
    float max_gamma_scaled = (float)MAX_BRIGHTNESS_PERCENT / 100.0f;
    if (gamma_scaled > max_gamma_scaled) {
        gamma_scaled = max_gamma_scaled;
    }

    uint32_t total_duty = (uint32_t)(gamma_scaled * PWM_MAX_DUTY);
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
        uint8_t ws_brightness = (uint8_t)(gamma_scaled * 255.0f + 0.5f);
        uint8_t red  = (1.0f - temp_ratio) * ws_brightness;
        uint8_t blue = temp_ratio * ws_brightness;
        ws2812_set_rgb(red, 0, blue);
    } else {
        ws2812_set_rgb(0, 0, 0);
    }
}

// ---------------------------- ADC helpers (ALS + NTC) --------------------------
static adc_oneshot_unit_handle_t adc1_handle;
static adc_oneshot_unit_handle_t adc2_handle;
static const adc_channel_t als_channel = ADC_CHANNEL_4;   // GPIO5 (verify on S3)
static const adc_channel_t ntc_channel = ADC_CHANNEL_5;   // GPIO16 (verify)

static void adc_init(void)
{
    // ADC1 for ALS
    adc_oneshot_unit_init_cfg_t cfg1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&cfg1, &adc1_handle));

    adc_oneshot_chan_cfg_t chan_cfg = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten    = ADC_ATTEN_DB_12
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
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    ESP_ERROR_CHECK(adc_cali_create_scheme_curve_fitting(&cali_config, &adc2_cali_handle));
}

static float read_als_lux(void)
{
    int raw = 0;
    esp_err_t err = adc_oneshot_read(adc1_handle, als_channel, &raw);
    if (err != ESP_OK) {
        return 0.0f;  // Return safe default on error
    }
    float voltage = (raw / 4095.0f) * 3.3f;
    float iph_uA  = (voltage / 10000.0f) * 1e6f;
    return (iph_uA / 100.0f) * 1000.0f; // lux
}

static float read_ntc_temperature(void)
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

    OTL_LOGI("sensor", "NTC raw=%d, V=%.3f, R=%.1f", raw, v, r);

    if (r <= 0) return NAN;

    float st = logf(r / nominal_res) / b_coeff + 1.0f / (nominal_temp + 273.15f);
    return (1.0f / st) - 273.15f;
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

        // Poll raw values and generate edge events (rising edges) for each pad.
        for (int i = 0; i < PAD_COUNT; ++i) {
            uint32_t raw = 0;
            touch_pad_read_raw_data(touch_pads[i], &raw);
            pressed_now[i] = (raw > threshold[i]);

            // Rising edge -> treat as a new tap/press event.
            if (pressed_now[i] && !prev_pressed[i]) {
                pad_intr |= (1UL << touch_pads[i]);
            }
        }

        // --- LED ON/OFF + optional NeoPixel enable on long-hold ---
        // NeoPixel is always off by default. To enable it, turn the light on
        // from OFF and keep holding the power pad for POWER_HOLD_NEOPIXEL_MS.
        bool power_pressed = pressed_now[0];
        bool power_pressed_edge = power_pressed && !prev_pressed[0];

        if (power_pressed_edge) {
            power_press_ms = now;

            if (!led_state) {
                led_state = true;
                neopixel_enabled = false;
                power_arm_neopixel = true;
                update_outputs();
                OTL_LOGI("touch", "LED turned: ON");
            } else {
                led_state = false;
                neopixel_enabled = false;
                power_arm_neopixel = false;
                update_outputs();
                OTL_LOGI("touch", "LED turned: OFF");
            }
        }

        if (power_pressed && power_arm_neopixel && !neopixel_enabled &&
            (now - power_press_ms) >= POWER_HOLD_NEOPIXEL_MS) {
            neopixel_enabled = true;
            update_outputs();
            OTL_LOGI("touch", "NeoPixel enabled");
        }

        if (!power_pressed) {
            power_arm_neopixel = false;
        }

        // --- Brightness UP ---
        if (pad_intr & (1UL << touch_pads[1])) {
            led_state = true; // Ensure LED is ON when changing brightness
            
            // First press or double-tap
            if (!button_being_held[1]) {
                button_being_held[1] = true;
                button_press_time[1] = now;
                last_repeat_time[1] = now;

                if (now - tap_last_ms[1] < DOUBLE_TAP_MS) {
                    brightness_percent = MAX_BRIGHTNESS_PERCENT;
                    update_outputs();
                    OTL_LOGI("touch", "Brightness set to MAX (%d%%)", MAX_BRIGHTNESS_PERCENT);
                    tap_last_ms[1] = now;
                } else {
                    float prev_brightness = brightness_percent;
                    brightness_percent = fminf(brightness_percent + (float)BRIGHT_STEP, (float)MAX_BRIGHTNESS_PERCENT);
                    if (brightness_percent != prev_brightness) {
                        update_outputs();
                        OTL_LOGI("touch", "Brightness increased to: %d%%", (int)lroundf(brightness_percent));
                    }
                    tap_last_ms[1] = now;
                }
                  
            }
        }

        // --- Brightness DOWN ---
        if (pad_intr & (1UL << touch_pads[2])) {
            // If the light is currently OFF, start from the normal minimum so a
            // dim-down press never "flashes" on at a previously-saved high
            // brightness.
            bool was_off = !led_state;
            if (was_off) {
                brightness_percent = MIN_BRIGHTNESS_PERCENT;
            }
            led_state = true;
            if (was_off) {
                update_outputs();
            }
             
            if (!button_being_held[2]) {
                button_being_held[2] = true;
                button_press_time[2] = now;
                last_repeat_time[2] = now;

                if ((brightness_percent >= MIN_BRIGHTNESS_PERCENT) && (now - tap_last_ms[2] < DOUBLE_TAP_MS)) {
                    brightness_percent = MIN_BRIGHTNESS_PERCENT;
                    update_outputs();
                    OTL_LOGI("touch", "Brightness set to MIN (%d%%)", (int)lroundf(MIN_BRIGHTNESS_PERCENT));
                    tap_last_ms[2] = now;
                } else {
                    float prev_brightness = brightness_percent;
                    float min_limit = (brightness_percent > MIN_BRIGHTNESS_PERCENT) ? MIN_BRIGHTNESS_PERCENT : HOLD_MIN_BRIGHTNESS_PERCENT;
                    brightness_percent = fmaxf(brightness_percent - (float)BRIGHT_STEP, min_limit);
                    if (brightness_percent != prev_brightness) {
                        update_outputs();
                        OTL_LOGI("touch", "Brightness decreased to: %d%%", (int)lroundf(brightness_percent));
                    }
                    tap_last_ms[2] = now;
                }
                  
            }
        }

        // --- Temp UP (cooler) ---
        if ((pad_intr & (1UL << touch_pads[3])) && led_state) {
            if (!button_being_held[3]) {
                button_being_held[3] = true;
                button_press_time[3] = now;
                last_repeat_time[3] = now;
                 
                if (now - tap_last_ms[3] < DOUBLE_TAP_MS) {
                    temp_ratio = 1.0f;
                    update_outputs();
                    OTL_LOGI("touch", "Temperature set to MAX COOL (1.0)");
                } else {
                    float prev_temp = temp_ratio;
                    temp_ratio = fminf(temp_ratio + TEMP_STEP, 1.0f);
                    if (temp_ratio != prev_temp) {
                        update_outputs();
                        OTL_LOGI("touch", "White temp increase: %.2f | Warm temp decrease: %.2f", 
                                temp_ratio, 1.0f - temp_ratio);
                    }
                }
                tap_last_ms[3] = now;
            }
        }

        // --- Temp DOWN (warmer) ---
        if ((pad_intr & (1UL << touch_pads[4])) && led_state) {
            if (!button_being_held[4]) {
                button_being_held[4] = true;
                button_press_time[4] = now;
                last_repeat_time[4] = now;
                 
                if (now - tap_last_ms[4] < DOUBLE_TAP_MS) {
                    temp_ratio = 0.0f;
                    update_outputs();
                    OTL_LOGI("touch", "Temperature set to MAX WARM (0.0)");
                } else {
                    float prev_temp = temp_ratio;
                    temp_ratio = fmaxf(temp_ratio - TEMP_STEP, 0.0f);
                    if (temp_ratio != prev_temp) {
                        update_outputs();
                        OTL_LOGI("touch", "Cool temp decrease: %.2f | Warm temp increase: %.2f", 
                                temp_ratio, 1.0f - temp_ratio);
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

                        float effective_repeat_ms = base_repeat_ms;
                        float dist_to_limit = 0.0f;
                        float min_brightness_limit = (i == 2) ? HOLD_MIN_BRIGHTNESS_PERCENT : MIN_BRIGHTNESS_PERCENT;
                        if (i == 1) {
                            dist_to_limit = (float)MAX_BRIGHTNESS_PERCENT - brightness_percent;
                        } else {
                            dist_to_limit = brightness_percent - min_brightness_limit;
                            if (dist_to_limit < 0.0f) {
                                dist_to_limit = 0.0f;
                            }
                        }
                        float end_min_ms = brightness_end_min_repeat_ms(dist_to_limit);
                        if (end_min_ms > effective_repeat_ms) {
                            effective_repeat_ms = end_min_ms;
                        }

                        float deep_min_ms = brightness_deep_dim_min_repeat_ms(brightness_percent);
                        if (deep_min_ms > effective_repeat_ms) {
                            effective_repeat_ms = deep_min_ms;
                        }

                        float rate_percent_per_s = ((float)BRIGHT_STEP * 1000.0f) / effective_repeat_ms;
                        float delta_percent = rate_percent_per_s * ((float)dt_ms / 1000.0f);
                        if (delta_percent <= 0.0f) {
                            break;
                        }

                        float prev_brightness = brightness_percent;
                        if (i == 1) {
                            brightness_percent = fminf(brightness_percent + delta_percent, (float)MAX_BRIGHTNESS_PERCENT);
                        } else {
                            brightness_percent = fmaxf(brightness_percent - delta_percent, min_brightness_limit);
                        }

                        if (brightness_percent != prev_brightness) {
                            led_state = true;
                            update_outputs();
                        }
                        break;
                    }

                    case 3: // Temp UP
                    case 4: // Temp DOWN
                    {
                        if (!led_state) {
                            break;
                        }

                        uint32_t dt_ms = now - last_repeat_time[i];
                        last_repeat_time[i] = now;

                        // Don't let the effective repeat be faster than our loop cadence.
                        float effective_repeat_ms = base_repeat_ms;
                        if (effective_repeat_ms < (float)dt_ms) {
                            effective_repeat_ms = (float)dt_ms;
                        }

                        float dist_to_limit = (i == 3) ? (1.0f - temp_ratio) : temp_ratio;
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

                        float prev_temp = temp_ratio;
                        if (i == 3) {
                            temp_ratio = fminf(temp_ratio + delta_ratio, 1.0f);
                        } else {
                            temp_ratio = fmaxf(temp_ratio - delta_ratio, 0.0f);
                        }

                        if (temp_ratio != prev_temp) {
                            update_outputs();
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
    // Configure initial state from pin level + last_sample on both candidate OUT pins
    int level15 = gpio_get_level(RADAR_OUT_GPIO);
    int level14 = gpio_get_level(RADAR_OUT_GPIO_ALT);
    bool out_state = (level14 == 1) || (level15 == 1);
    bool moving = radar_last_sample.valid && (radar_last_sample.targetType & 0x01);
    bool stationary = radar_last_sample.valid && (radar_last_sample.targetType & 0x02);
    radar_presence = out_state || moving || stationary;
    radar_occupied = radar_presence;

    uint32_t last_log_ms      = 0;
    uint32_t last_presence_ms = esp_timer_get_time() / 1000;
    uint32_t presence_candidate_since_ms = 0;
    bool     presence_candidate_active   = false;

    while (1) {
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
            OTL_LOGI("radar",
                     "presence=%s OUT14=%d OUT15=%d moving=%d stationary=%d movDist=%ucm statDist=%ucm",
                     (radar_presence ? "ON" : "OFF"),
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
        if (new_presence && !radar_occupied &&
            presence_candidate_active &&
            (now_ms - presence_candidate_since_ms) > RADAR_PRESENCE_ON_DELAY_MS) {
            radar_presence = true;
            handle_occupancy_gained();
            radar_occupied = true;
        } else if (!new_presence && radar_occupied &&
                   (now_ms - last_presence_ms) > RADAR_ABSENCE_TIMEOUT_MS) {
            radar_presence = false;
            handle_occupancy_lost();
            radar_occupied = false;
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
#endif

// touch_debug_task removed - was only for debugging and was disabled

// ---------------------------- Sensor task (ALS + NTC) -------------------------
static void sensor_task(void *arg)
{
    while (1) {
        float lux  = read_als_lux();
        float temp = read_ntc_temperature();
        OTL_LOGI("sensor", "Ambient %.0f lux | Temperature %.1f °C", lux, temp);
        vTaskDelay(pdMS_TO_TICKS(SENSOR_READ_INTERVAL_MS));
    }
}

// ---------------------------- app_main ----------------------------------------
// Remove all timer-related code from app_main()

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

    // Calibrate touch pads using Arduino-style baseline/thresholds
    calibrate_touch_pads();

    // Using polling-based touch handling in touch_task() instead of ISR

    // 5. ADC (ALS + NTC)
    adc_init();

    // 6. Start tasks with adequate stack sizes
    // touch_task and radar_task need larger stacks due to complex state
    // sensor_task and recalib_task are simpler and can use smaller stacks
    xTaskCreate(touch_task, "touch_task", 4096, NULL, 5, NULL);
#if CONFIG_OTL_SERIAL_OUTPUT
    xTaskCreate(sensor_task, "sensor_task", 2560, NULL, 5, NULL);
#endif
    xTaskCreate(recalibration_task, "recalib_task", 2560, NULL, 2, NULL);
#if CONFIG_OTL_PRESENCE_SENSOR
    xTaskCreate(radar_task, "radar_task", 4096, NULL, 4, NULL);
#endif

    // Initial output state
    update_outputs();
    
    OTL_LOGI("main", "Open Task Light initialization complete");
}
