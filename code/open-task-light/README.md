| Supported Targets | ESP32 | ESP32-C2 | ESP32-C3 | ESP32-C6 | ESP32-H2 | ESP32-P4 | ESP32-S2 | ESP32-S3 | Linux |
| ----------------- | ----- | -------- | -------- | -------- | -------- | -------- | -------- | -------- | ----- |

# Hello World Example

Starts a FreeRTOS task to print "Hello World".

(See the README.md file in the upper level 'examples' directory for more information about examples.)

## Open Task Light build options

Configure via `idf.py menuconfig` → `Open Task Light`:

- `Enable serial log output` (`CONFIG_OTL_SERIAL_OUTPUT`)
- `Enable presence sensor (LD2410B)` (`CONFIG_OTL_PRESENCE_SENSOR`)

## ESPHome (Home Assistant)

A starter ESPHome config (CWWW light + 5 touch pads + optional LD2410B) lives at `esphome/open-task-light.yaml`.

- Copy `esphome/secrets.example.yaml` to `esphome/secrets.yaml` and fill in your Wi-Fi credentials.
- Set `wifi.power_save_mode: none` to reduce Home Assistant control latency.
- Set `logger.baud_rate: 0` to disable serial logs (or keep `115200` for debugging).
- Use the `${friendly_name} Touch Controls` switch to quickly disable touch actions while tuning thresholds.
- Touch thresholds can be auto-calibrated at boot and via the `${friendly_name} Calibrate Touch` button.
- Optional: set `esp32_touch.setup_mode: true` temporarily to tune touch `threshold` values (ESP32-S3 touch values increase on touch).
- Optional: uncomment the LD2410B section in the YAML to enable presence sensing.

## How to use example

Follow detailed instructions provided specifically for this example.

Select the instructions depending on Espressif chip installed on your development board:

- [ESP32 Getting Started Guide](https://docs.espressif.com/projects/esp-idf/en/stable/get-started/index.html)
- [ESP32-S2 Getting Started Guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s2/get-started/index.html)


## Example folder contents

The project **hello_world** contains one source file in C language [hello_world_main.c](main/hello_world_main.c). The file is located in folder [main](main).

ESP-IDF projects are built using CMake. The project build configuration is contained in `CMakeLists.txt` files that provide set of directives and instructions describing the project's source files and targets (executable, library, or both).

Below is short explanation of remaining files in the project folder.

```
├── CMakeLists.txt
├── pytest_hello_world.py      Python script used for automated testing
├── main
│   ├── CMakeLists.txt
│   └── hello_world_main.c
└── README.md                  This is the file you are currently reading
```

For more information on structure and contents of ESP-IDF projects, please refer to Section [Build System](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/build-system.html) of the ESP-IDF Programming Guide.

## Troubleshooting

* Program upload failure

    * Hardware connection is not correct: run `idf.py -p PORT monitor`, and reboot your board to see if there are any output logs.
    * The baud rate for downloading is too high: lower your baud rate in the `menuconfig` menu, and try again.

## Technical support and feedback

Please use the following feedback channels:

* For technical queries, go to the [esp32.com](https://esp32.com/) forum
* For a feature request or bug report, create a [GitHub issue](https://github.com/espressif/esp-idf/issues)

We will get back to you as soon as possible.
