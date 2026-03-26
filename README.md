# Open Task Light

Open Task Light is an ESP32-S3 task light project with firmware in
`code/open-task-light/` and hardware reference files at the repo root.

The current native firmware supports:
- capacitive touch controls for power, brightness, and color temperature
- separate warm and cool LED channels
- a WS2812 status LED
- optional LD2410B presence sensing
- optional Wi-Fi with native Home Assistant MQTT integration
- optional Apple Home support through native HomeKit
- optional Wi-Fi/SNTP-driven circadian color temperature control

The repo is organized as a top-level project folder with the firmware living
under `code/open-task-light/` and the hardware reference files at the root.

## Repo Layout

- `code/open-task-light/` firmware project
- `pcb/` PCB PDFs and fabrication reference files
- `LICENSE` project license

## Firmware

The active firmware project is in `code/open-task-light/`.

That subproject currently includes:
- ESP-IDF native firmware
- ESPHome configuration
- Arduino sketches
- checked-in defaults in `sdkconfig.defaults`

The ESP-IDF path is the main day-to-day firmware now. It defaults to an
offline local-light build. Wi-Fi, MQTT, HomeKit, and circadian are all
optional compile-time features.

Start with the detailed firmware README here:
- `code/open-task-light/README.md`

For local build profile notes, see:
- `code/open-task-light/local-config-profiles.md`

## Hardware Files

The `pcb/` folder currently contains:
- `DIYson.PDF`
- `DIYson_Fab.PDF`
- `Bill of Materials-DIYson.csv`

These are the board reference and fabrication outputs for the light.

For current build details and project updates, see the Basement Labs page:
- <https://www.basementlabs.net/open-task-light-2>

## Getting Started

If you are working on firmware:

```sh
cd code/open-task-light
```

Then follow the build and setup instructions in `code/open-task-light/README.md`.

Local build outputs such as `build/` and `sdkconfig` are generated there and
are intentionally not tracked in Git.

If you are working on hardware:
- review the PDFs in `pcb/`
- use the Basement Labs page for the current parts/build reference
- keep firmware pin mappings aligned with the board wiring

## Notes

The repo-root README is intentionally a project index. The detailed
day-to-day instructions for building, configuring, and using the embedded code
live in the nested firmware README.
