# Repository Guidelines

## Project Structure & Module Organization

- `relod_v4_0/` contains the current production firmware (`relod_v4_0.ino`).
- `libraries/` holds all bundled Arduino libraries required by the firmware.
- `relod_*/` and `sketch_*/` folders are historical iterations; avoid editing unless you are validating legacy behavior.
- `relod_v4_0/build/` is a local build output directory; do not commit generated artifacts.

## Build, Test, and Development Commands

- `arduino-cli compile --fqbn esp32:esp32:esp32 relod_v4_0/` builds the firmware for the ESP32 target.
- `arduino-cli upload --fqbn esp32:esp32:esp32 -p /dev/ttyUSB0 relod_v4_0/` flashes to a connected device (adjust port).
- Arduino IDE is supported if you prefer GUI workflows; open `relod_v4_0/relod_v4_0.ino`.

## Coding Style & Naming Conventions

- Indentation: 2 spaces, no tabs; follow the existing Arduino/C++ style in `relod_v4_0/relod_v4_0.ino`.
- Constants and macros are `UPPER_SNAKE_CASE`; function names are `lowerCamelCase`.
- Keep serial output disabled in production paths unless explicitly needed for debugging.

## Testing Guidelines

- No automated test suite exists. Validate changes on hardware.
- When modifying sensor reads or deep-sleep behavior, run at least one full wake → sample → POST → sleep cycle.
- If adding new behavior, include a brief manual test note in the PR description.

## Commit & Pull Request Guidelines

- Commit messages follow an imperative, capitalized style (e.g., “Add”, “Update”, “Fix”).
- PRs should include: a concise summary, linked issue (if any), hardware tested, and any logs or observations.
- If you change network endpoints or OTA behavior, call out the risk and rollback plan.

## Configuration & Security Notes

- WiFi provisioning uses WiFiManager captive portal; avoid hardcoding credentials.
- OTA updates and telemetry target `relod.fly.dev`; confirm URLs before releasing.
