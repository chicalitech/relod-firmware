# OTA Safety

Firmware checks:

```text
https://relod.fly.dev/latest_firmware
```

## Current Expected Metadata

```json
{
  "version": "6.1.0",
  "url": "https://example.com/firmware.bin",
  "sha256": "64 lowercase or uppercase hex characters",
  "force_update": false
}
```

`hash` is accepted as a temporary alias for `sha256`.

## Update Rules

The device only applies an update when:

1. metadata contains a version and URL,
2. metadata contains a valid SHA-256 hex digest,
3. the metadata version is newer than `CURRENT_FIRMWARE_VERSION`, or `force_update` is true and the version differs,
4. the binary downloads successfully,
5. the downloaded binary SHA-256 matches metadata,
6. `Update.end()` and `Update.isFinished()` both succeed.

If the server advertises stale firmware such as `5.0` while the device is on `6.0`, the device does not downgrade.

## Known Transport Caveat

The OTA download path still uses `WiFiClientSecure::setInsecure()` until CA validation or certificate pinning is validated on real XIAO ESP32-C6 hardware. SHA-256 verification is mandatory as the current safety backstop.

Do not relax hash verification to recover old OTA behavior. Fix the metadata endpoint instead.

## Rollout Guardrails

- Never broad-roll OTA metadata without hardware-owner review.
- Keep a manual recovery path documented for any release candidate.
- Prefer staged rollout metadata on the server before enabling a fleet-wide update.
- The API-side firmware metadata endpoint must be corrected before relying on OTA in production.
