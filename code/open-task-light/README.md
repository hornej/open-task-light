# Open Task Light

Firmware and configs for the Open Task Light: an ESP32-S3 task light with
capacitive touch controls, warm/cool PWM channels, a WS2812 status LED, and
optional LD2410B presence sensing.

There are three supported build paths in this repo:
- ESP-IDF (native firmware)
- ESPHome (Home Assistant)
- Arduino (standalone sketch)

## Build options

### ESP-IDF (native firmware)
Source: `main/otl_main.c`

Quick start:
```sh
idf.py set-target esp32s3
idf.py menuconfig
idf.py build
idf.py -p PORT flash monitor
```

Menuconfig options under `Open Task Light`:
- `Enable serial log output` (`CONFIG_OTL_SERIAL_OUTPUT`)
- `Enable periodic status logs (every 10s)` (`CONFIG_OTL_LOG_STATUS`)
- `Enable sensor debug logs (raw ADC)` (`CONFIG_OTL_SENSOR_DEBUG`)
- `Enable touch UI event logs` (`CONFIG_OTL_LOG_TOUCH_EVENTS`)
- `Enable touch calibration logs` (`CONFIG_OTL_LOG_TOUCH_CAL`)
- `Enable raw touch pad value logs` (`CONFIG_OTL_LOG_TOUCH_RAW`) + `Raw touch log interval (ms)` (`CONFIG_OTL_TOUCH_RAW_LOG_INTERVAL_MS`)
- `Enable PWM duty logs during transitions` (`CONFIG_OTL_LOG_PWM_DUTY`) + `PWM duty log interval (ms)` (`CONFIG_OTL_PWM_LOG_INTERVAL_MS`)
- `Enable presence sensor status logs` (`CONFIG_OTL_LOG_RADAR_STATUS`)
- `Enable presence sensor (LD2410B)` (`CONFIG_OTL_PRESENCE_SENSOR`)
- `Non-overlapping warm/cool PWM` (`CONFIG_OTL_NONOVERLAP_PWM`)
- `Enable circadian color temperature (WiFi + SNTP)` (`CONFIG_OTL_CIRCADIAN_ENABLE`)

Circadian notes:
- Set `WiFi SSID`, `WiFi Password`, and a POSIX `Timezone` string (examples: `PST8PDT,M3.2.0/2,M11.1.0/2`, `EST5EDT,M3.2.0/2,M11.1.0/2`, `UTC0`).
- The firmware syncs time via SNTP and updates the warm/cool mix smoothly across the day (cooler midday, warmer evenings/night).
- When enabled, the temperature touch buttons act as a user offset on top of the circadian schedule.

Notes:
- This firmware mirrors the Arduino sketch and is a structured starting point.
  Verify pin mapping and touch thresholds for your specific board.

### ESPHome (Home Assistant)
Config: `esphome/open-task-light.yaml`

Quick start:
```sh
cp esphome/secrets.example.yaml esphome/secrets.yaml
esphome run esphome/open-task-light.yaml
```

Notes:
- Set `wifi.power_save_mode: none` to reduce command latency.
- Set `logger.baud_rate: 0` to disable serial logs (use `115200` for debugging).
- Use `use_address` if mDNS is unreliable in your network.
- Optional: uncomment the LD2410B section in the YAML to enable presence sensing.

### Arduino
Sketch: `arduino/otl.ino`

Optional radar test sketch: `arduino/LD2410B.ino`

Quick start:
1. Open `arduino/otl.ino` in the Arduino IDE.
2. Select an ESP32-S3 board and install the `Adafruit NeoPixel` library.
3. Build and upload.

## Default pin map

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

## Repo layout

- `arduino/` Arduino sketches
- `esphome/` ESPHome configuration
- `main/` ESP-IDF firmware
