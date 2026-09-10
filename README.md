# Relod Firmware

Production firmware for deployed Relod inventory containers.

The active target is `seeed_xiao_esp32c6` with Arduino through PlatformIO. The firmware samples:

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
platformio run
```

Upload to a connected XIAO ESP32-C6:

```sh
platformio run --target upload
```

Open a serial monitor:

```sh
platformio device monitor -b 115200
```

## Arduino IDE

The active sketch for hardware review is:

```text
relod_firmware/relod_firmware.ino
```

Open that `.ino` in Arduino IDE if you need the sketch workflow. Select the Seeed Studio XIAO ESP32-C6 board and make the libraries in `libraries/` available to Arduino IDE before compiling.

## Project Layout

- `relod_firmware/relod_firmware.ino` - production firmware sketch and entrypoint.
- `platformio.ini` - active XIAO ESP32-C6 build configuration; PlatformIO builds the sketch folder directly.
- `libraries/` - vendored Arduino libraries required by the firmware.
- `docs/measurement-payload.md` - current payload contract.
- `docs/ota-safety.md` - OTA update guardrails.
- `hello_world_test/` - separate toolchain smoke test for XIAO ESP32-C6.

Historical sketches and old ESP-IDF files remain for archaeology only. Treat `relod_firmware/relod_firmware.ino` and `platformio.ini` as the production path unless hardware owners say otherwise.

The active partition scheme is `ota_nofs_4MB.csv`: two OTA-capable app slots and no filesystem partition. The firmware does not use SPIFFS/LittleFS, and this keeps enough room for the Arduino ESP32 3.x image while preserving OTA updates.

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

No local command can prove real sensor behavior. Any hardware-facing release still needs a physical XIAO ESP32-C6 validation pass before rollout.
