# AGENTS.md

This repo controls firmware for deployed Relod devices. Treat it as hardware-adjacent production code.

## First Read

1. `README.md`
2. `docs/measurement-payload.md`
3. `docs/ota-safety.md`
4. `platformio.ini`
5. `src/main.cpp`

## Operating Rules

- Work on a feature branch and open a PR. Do not push directly to `main`.
- Keep payload changes additive and optional. Existing deployed devices must keep working.
- Do not require auth, change provisioning, or alter sleep cadence unless a device rollout plan exists.
- Do not make OTA behavior more permissive. OTA must only apply newer/allowed firmware and must verify the downloaded binary.
- Keep generated build output out of source control.
- Prefer small, named helper functions over growing `loop()` with inline workflow code.
- Document every new payload field with units, producer semantics, and whether older devices omit it.
- Assume the server may ignore unknown fields. Firmware must not depend on immediate backend adoption.

## Commands

```sh
scripts/check
platformio run
platformio run --target upload
platformio device monitor -b 115200
```

If `platformio` is not installed, install PlatformIO Core before attempting firmware validation.

## Agent Workflow

1. Check `git status -sb` before editing.
2. Read the linked GitHub issue and this repo guidance.
3. Inspect the actual firmware and vendored library APIs before choosing function names.
4. Make the smallest hardware-safe source change that satisfies the issue.
5. Update docs and validation rails in the same PR when behavior or contracts change.
6. Run `scripts/check`; if hardware validation is unavailable, say that explicitly in the PR.
7. Open a PR with issue links, validation evidence, and hardware rollout caveats.

## Current Known Constraints

- The production source path is PlatformIO Arduino on `seeed_xiao_esp32c6`.
- `src/main.cpp` currently reports firmware version `6.0`.
- The measurement endpoint is `https://relod.fly.dev/measurement`.
- The firmware metadata endpoint is `https://relod.fly.dev/latest_firmware`.
- OTA still uses `WiFiClientSecure::setInsecure()` transport until certificate/CA handling is validated on device; SHA-256 verification is the required safety backstop in this repo.
