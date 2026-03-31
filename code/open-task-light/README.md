# Open Task Light

Firmware and configs for Open Task Light: an ESP32-S3 task light with
capacitive touch controls, warm/cool PWM channels, a WS2812 status LED, and
optional LD2410B presence sensing.

There are three supported build paths in this repo:
- ESP-IDF native firmware
- ESPHome configuration
- Arduino standalone sketch

The ESP-IDF firmware is the primary implementation and has the most complete
feature set.

## What The Light Does

Hardware and local behavior:
- 5 capacitive touch pads: power, brightness up, brightness down, cooler, warmer
- 2 PWM LED channels: warm white and cool white
- WS2812 status LED
- optional LD2410B presence sensor
- ambient light sensor
- LED NTC temperature sensor
- ESP32-S3 internal temperature monitoring

Optional connected behavior in the native firmware:
- shared Wi-Fi station support
- Home Assistant via native MQTT discovery
- Apple Home via native HomeKit
- Wi-Fi + SNTP circadian color temperature schedule

## Using The Light

### Touch controls

- Power pad:
  - tap toggles the light on or off
  - if the light is off, pressing power turns it on immediately
  - if you keep holding the power pad for about 3 seconds while turning on,
    the WS2812 status LED is enabled for that on-cycle
- Brightness up pad:
  - single tap raises brightness by 1%
  - double tap jumps to max brightness
  - hold ramps brightness upward smoothly
- Brightness down pad:
  - single tap lowers brightness by 1%
  - double tap jumps to the normal minimum brightness
  - hold ramps brightness downward
  - if you want the deepest dim range below the normal minimum, release and
    hold dim-down again after reaching minimum
- Cooler pad:
  - single tap shifts color temperature cooler
  - double tap jumps to maximum cool
  - hold ramps cooler
- Warmer pad:
  - single tap shifts color temperature warmer
  - double tap jumps to maximum warm
  - hold ramps warmer

### Presence behavior

If the LD2410B presence sensor is installed and enabled:
- occupancy turns on only after continuous presence for the configured on delay
- occupancy turns off only after continuous absence for the configured timeout
- when occupancy clears, the light fades out
- when occupancy returns, the light fades back to its previous state

### Thermal behavior

- LED NTC temperature can clamp the maximum allowed brightness
- ESP32-S3 internal temperature is monitored separately as a chip-side failsafe
- the LED thermal limit is runtime-adjustable from Home Assistant when MQTT is enabled

## ESP-IDF Native Firmware

Source: `main/otl_main.c`

Quick start:

```sh
idf.py set-target esp32s3
idf.py menuconfig
idf.py build
idf.py -p PORT flash monitor
```

Checked-in defaults live in `sdkconfig.defaults`. Your `menuconfig` changes are
written to the local `sdkconfig` file, which is intentionally ignored so
Wi-Fi credentials and other machine-specific settings stay out of Git.

Default behavior:
- `sdkconfig.defaults` is an offline baseline
- Wi-Fi is off by default
- Home Assistant MQTT is off by default
- HomeKit is off by default
- circadian is off by default

That means a default build stays simple and local-only unless you explicitly
turn connected features on.

### Menuconfig areas

Under `Open Task Light`, the important sections are:

- `Serial logging`
  - app logs, periodic status, touch logs, calibration logs, PWM logs
- `Connectivity`
  - `Enable WiFi`
  - `Enable Home Assistant via MQTT`
  - `Enable Apple Home via HomeKit`
- `Presence sensor`
  - LD2410B enable, optional first-boot Bluetooth disable, distance thresholds,
    on/off delay, polling interval
- `PWM`
  - non-overlapping warm/cool PWM
- `Circadian lighting`
  - Wi-Fi/SNTP-based circadian schedule and compile-time defaults

### Connectivity

The native firmware uses a shared Wi-Fi subsystem in `components/otl_net/`.
MQTT, HomeKit, and circadian all sit on top of that and are only compiled when
enabled.

Wi-Fi notes:
- Wi-Fi credentials are compile-time settings in `menuconfig`
- MQTT and HomeKit wait for Wi-Fi to connect before starting
- if Wi-Fi is disabled, none of the network stacks are initialized

### Home Assistant via MQTT

Enable in `menuconfig`:
- `Enable WiFi`
- `Enable Home Assistant via MQTT`

Then set:
- `MQTT broker URI`
- `MQTT username` and `MQTT password` if your broker requires auth
- optional discovery prefix, topic base, and device name

The firmware publishes a native Home Assistant MQTT device with:

- light entity
  - on/off
  - brightness
  - color temperature
- occupancy binary sensor when presence is enabled
- telemetry and diagnostics
  - ambient lux
  - LED NTC temperature
  - ESP32-S3 internal temperature
  - thermal max brightness cap
  - thermal-limited status
  - Wi-Fi RSSI
  - last event
- runtime controls
  - LED thermal limit
  - circadian lighting enable
  - circadian lighting coolest time
  - circadian lighting warmest time
  - circadian morning ramp duration
  - verbose diagnostics

Important behavior:
- the light state stays synchronized across touch, MQTT, presence behavior, and
  HomeKit when multiple integrations are enabled
- circadian lighting is runtime-toggleable from Home Assistant so it does not
  have to fight Adaptive Lighting or your own automations
- the MQTT integration also publishes a structured event stream and a `Last Event`
  text sensor instead of mirroring raw serial logs

### Apple Home via HomeKit

Enable in `menuconfig`:
- `Enable WiFi`
- `Enable Apple Home via HomeKit`

Then set:
- `Accessory name`
- `Setup code`
- `Setup ID`

Pairing flow:
1. enter Wi-Fi credentials in `menuconfig`
2. build and flash the firmware
3. boot the light onto Wi-Fi
4. pair it in Apple Home using the configured setup code

Default development values:
- setup code: `111-22-333`
- setup ID: `OTL1`

Change those before handing a build to anyone else.

Current HomeKit surface:
- Lightbulb service
  - on/off
  - brightness
  - color temperature
- Occupancy Sensor service when presence is enabled

HomeKit intentionally stays narrower than Home Assistant. Diagnostics and
runtime tuning are exposed through MQTT/HA, not Apple Home.

### Circadian lighting

Enable in `menuconfig`:
- `Enable WiFi`
- `Enable circadian color temperature (WiFi + SNTP)`

Compile-time circadian settings:
- `Timezone (POSIX TZ string)`
- `SNTP server`
- `Update interval`
- `Coolest time (HH:MM)`
- `Warmest time (HH:MM)`
- `Morning cool ramp duration (minutes)`
- `Minimum cool ratio (%)`
- `Maximum cool ratio (%)`

Runtime behavior:
- the circadian engine computes a base warm/cool mix across the day
- after the warmest time, the light can hold maximum warmth overnight and only
  start cooling during the configurable morning ramp window before the coolest
  time
- temperature touch controls act as a user offset on top of that base
- if MQTT is enabled, Home Assistant can turn circadian lighting on or off at runtime
- if you want Home Assistant to own color temperature policy, disable circadian
  lighting and use your own automations or Adaptive Lighting

## Local Configuration Profiles

If you keep local config fragments outside Git, use:

```sh
./tools/use-local-profile.sh prod
./tools/use-local-profile.sh debug
```

That rebuilds `sdkconfig` from:
1. `sdkconfig.defaults`
2. `sdkconfig.local.shared`
3. `sdkconfig.local.<profile>`

See `local-config-profiles.md` for the expected pattern.

## ESPHome

Config: `esphome/open-task-light.yaml`

Quick start:

```sh
cp esphome/secrets.example.yaml esphome/secrets.yaml
esphome run esphome/open-task-light.yaml
```

Notes:
- `esphome/secrets.yaml` is intentionally ignored; keep your real Wi-Fi
  credentials there
- this path still exists for people who want an ESPHome-based build
- the native ESP-IDF firmware is the preferred path if you want the full touch
  behavior implemented in this repo

## Arduino

Sketch: `arduino/otl/otl.ino`

Optional radar test sketch: `arduino/LD2410B.ino`

Quick start:
1. Open `arduino/otl/otl.ino` in the Arduino IDE.
2. Select an ESP32-S3 board and install the `Adafruit NeoPixel` library.
3. Build and upload.

## Default Pin Map

| Function | GPIO | Notes |
| --- | --- | --- |
| Touch power | GPIO7 | T7 |
| Touch brightness up | GPIO8 | T8 |
| Touch brightness down | GPIO9 | T9 |
| Touch temp up | GPIO10 | T10 |
| Touch temp down | GPIO11 | T11 |
| PWM warm | GPIO35 | LEDC |
| PWM cool | GPIO36 | LEDC |
| WS2812 data | GPIO40 | Status LED |
| ALS (ambient light) | GPIO5 | ADC |
| NTC thermistor | GPIO16 | ADC |
| LD2410B OUT | GPIO15 | Alt: GPIO14 |
| LD2410B UART TX | GPIO43 | ESP32 TX -> radar RX |
| LD2410B UART RX | GPIO44 | ESP32 RX <- radar TX |

Adjust pins to match your board wiring as needed.

## Repo Layout

- `arduino/` Arduino sketches
- `components/esp-homekit-sdk/` vendored HomeKit SDK
- `components/otl_circadian/` shared circadian controller
- `components/otl_net/` shared Wi-Fi subsystem
- `esphome/` ESPHome configuration
- `managed_components/` ESP-IDF managed dependencies
- `main/` ESP-IDF firmware
- `sdkconfig.defaults` checked-in ESP-IDF defaults
- `tools/` helper scripts such as the local profile switcher and PWM scope watcher
