# Relod Firmware

Production firmware for deployed Relod inventory containers.

Production uses the SparkFun ESP32-C6 Thing Plus in `relod_sparkfun_v8_0/`, with Arduino through PlatformIO. XIAO remains a testing target. The firmware samples:

- VL53L5CX 8x8 time-of-flight distance grid
- Si7021 temperature and humidity
- BMA400 acceleration and wake interrupts
- MAX1704x battery voltage, SOC, and alert state
- Three analog LED/rail voltage channels

## Safety Rules

- Deployed devices are live. Keep payload changes additive and optional.
- Do not require hard auth until device provisioning and rollout are planned.
- Do not remove existing payload fields without coordinating API and website changes.
- Do not ship OTA rollout behavior without hardware-owner review.
- API support should land before firmware fields are required by product UI.

## Quick Start

Run the repo check script. It uses `platformio` when installed and falls back to `uvx --from platformio platformio` when available:

```sh
scripts/check
```

Build only:

```sh
platformio run -d relod_sparkfun_v8_0 -e sparkfun_c6
```

Upload to a connected SparkFun ESP32-C6 Thing Plus:

```sh
platformio run -d relod_sparkfun_v8_0 -e sparkfun_c6 --target upload
```

Open a serial monitor:

```sh
platformio device monitor -d relod_sparkfun_v8_0 -b 115200
```

## Arduino IDE

The separate XIAO testing sketch is:

```text
relod_firmware/relod_firmware.ino
```

Open that `.ino` in Arduino IDE if you need the sketch workflow. Select the Seeed Studio XIAO ESP32-C6 board and make the libraries in `libraries/` available to Arduino IDE before compiling.

## Project Layout

- `relod_sparkfun_v8_0/src/main.cpp` - production SparkFun entrypoint.
- `relod_sparkfun_v8_0/platformio.ini` - production build configuration.
- `relod_sparkfun_v8_0/VERSION` - release version source.
- `docs/ota-release-runbook.md` - candidate publishing, hardware tests and promotion.
- `relod_firmware/relod_firmware.ino` and root `platformio.ini` - XIAO testing target.
- `libraries/` - vendored Arduino libraries required by the firmware.
- `docs/measurement-payload.md` - current payload contract.
- `docs/ota-safety.md` - OTA update guardrails.
- `hello_world_test/` - separate toolchain smoke test for XIAO ESP32-C6.

Historical sketches and old ESP-IDF files remain for reference. Use the explicit SparkFun project/environment for production builds.

SparkFun uses `default_16MB.csv` with two `0x640000`-byte OTA app slots. The separate XIAO testing target uses `ota_nofs_4MB.csv`.

## Current Payload Direction

The base payload is unchanged and now includes optional telemetry for:

- schema and firmware metadata
- wake/reset and boot counters
- WiFi/connect timing
- previous POST outcome
- heap and battery alert state
- compact VL53L5CX quality summaries
- BMA400 interrupt context

See `docs/measurement-payload.md` for field names, units, and caveats.

## Validation

Run `scripts/check` before opening a PR. CI also runs a PlatformIO build on every push and PR.

No local command can prove real sensor behavior. Production releases require a physical SparkFun validation pass before rollout. Follow the [release runbook](docs/ota-release-runbook.md).
