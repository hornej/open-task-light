# Open Task Light

Open Task Light is an ESP32-S3 task light project with firmware in
`code/open-task-light/` and hardware reference files at the repo root. The
current firmware supports capacitive touch controls, separate warm/cool LED
channels, a WS2812 status LED, optional LD2410B presence sensing, and optional
Wi-Fi/SNTP-driven circadian color temperature control.

This repo is organized as a top-level project folder with the firmware living
under `code/open-task-light/` and the hardware reference files at the repo root.

## Repo layout

- `code/open-task-light/` firmware project
- `pcb/` PCB PDFs and fabrication reference files
- `LICENSE` project license

## Firmware

The active firmware project is in `code/open-task-light/`.

That subproject currently includes:
- ESP-IDF firmware
- ESPHome configuration
- Arduino sketches
- checked-in defaults in `sdkconfig.defaults`

Start with the detailed firmware README here:
- `code/open-task-light/README.md`

## Hardware files

The `pcb/` folder currently contains:
- `DIYson.PDF`
- `DIYson_Fab.PDF`
- `Bill of Materials-DIYson.csv`

These are the board reference and fabrication outputs for the light.

For current build details and project updates, see the Basement Labs page:
- <https://www.basementlabs.net/open-task-light-2>

## Getting started

If you are working on firmware:

```sh
cd code/open-task-light
```

Then follow the build instructions in `code/open-task-light/README.md`.

Local build outputs such as `build/` and `sdkconfig` are generated there and
are intentionally not tracked in Git.

If you are working on hardware:
- review the PDFs in `pcb/`
- use the Basement Labs page for the current parts/build reference
- keep firmware pin mappings aligned with the board wiring

## Notes

The repo-root README is intentionally a project index. The detailed day-to-day
development instructions for the embedded code live in the nested firmware
README.
