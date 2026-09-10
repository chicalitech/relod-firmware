# Measurement Payload Contract

Firmware posts JSON to:

```text
https://relod.fly.dev/measurement
```

The payload must remain backward-compatible. Servers should accept missing optional fields because deployed devices may run older firmware for a long time.

## Existing Fields

| Field | Type | Units | Notes |
| --- | --- | --- | --- |
| `device_id` | string | MAC address | Station MAC from `esp_wifi_get_mac(WIFI_IF_STA)`. |
| `temperature` | number | deg C | Si7021 reading. |
| `humidity` | number | percent RH | Si7021 reading. |
| `voltage` | number | volts | MAX1704x battery voltage. |
| `soc` | number | percent | MAX1704x state of charge. |
| `voltage_red` | number | volts | Analog rail/LED proxy reading. |
| `voltage_yellow` | number | volts | Analog rail/LED proxy reading. |
| `voltage_green` | number | volts | Analog rail/LED proxy reading. |
| `firmware_version` | string | semver-like | Current firmware reports `6.0`. |
| `distance_mm` | number[] | millimeters | 64-zone VL53L5CX grid, same order as existing firmware. |
| `acceleration_x` | number | g | BMA400 latest accel sample. |
| `acceleration_y` | number | g | BMA400 latest accel sample. |
| `acceleration_z` | number | g | BMA400 latest accel sample. |

## Additive Health Fields

| Field | Type | Units | Notes |
| --- | --- | --- | --- |
| `schema_version` | number | none | Optional firmware payload schema. Starts at `1`. |
| `boot_count` | number | boots | RTC-retained count across deep sleep. |
| `measurement_sequence` | number | samples | RTC-retained measurement sequence. |
| `sleep_interval_s` | number | seconds | Current deep-sleep timer interval. |
| `wake_cause` | string | enum | ESP sleep wake cause such as `timer`, `gpio`, `ext0`, `ext1`. |
| `reset_reason` | string | enum | ESP reset reason such as `poweron`, `deep_sleep`, `brownout`. |
| `motion_wake` | boolean | none | True when wake cause or ISR indicates motion/GPIO wake. |
| `battery_alert` | boolean | none | MAX1704x alert flag. |
| `wifi_rssi_dbm` | number | dBm | WiFi RSSI after connection. |
| `wifi_connect_ms` | number | milliseconds | Time spent in WiFiManager connect/provisioning path. |
| `http_status` | number | HTTP status | Previous completed POST status. The current status is unknowable until after the payload is sent. |
| `post_ms` | number | milliseconds | Previous completed POST duration. |
| `retry_count` | number | attempts | Previous POST retry count. |
| `free_heap` | number | bytes | Free heap at serialization time. |

## Distance Quality Fields

| Field | Type | Units | Notes |
| --- | --- | --- | --- |
| `distance_data_ready` | boolean | none | Result of `VL53L5CX.isDataReady()`. |
| `distance_read_ok` | boolean | none | True when `getRangingData()` succeeded. |
| `distance_valid_count` | number | zones | Count of zones with positive, non-far, usable-status distance. |
| `distance_zero_count` | number | zones | Count of zones with distance `<= 0`. |
| `distance_far_count` | number | zones | Count of zones with distance `>= 3500mm`. |
| `distance_bad_status_count` | number | zones | Count of zones outside the current usable VL53L5CX status set. |
| `distance_spread_mm` | number | millimeters | Approximate p90-p10 spread over valid zones. |
| `distance_target_detected_count` | number | zones | Count of zones reporting one or more targets. |
| `distance_signal_avg` | number | sensor units | Mean `signal_per_spad`. |
| `distance_ambient_avg` | number | sensor units | Mean `ambient_per_spad`. |
| `distance_reflectance_avg` | number | percent | Mean reflectance across zones. |

## Motion Context Fields

| Field | Type | Units | Notes |
| --- | --- | --- | --- |
| `bma400_interrupt_status` | number | bitmask | Raw BMA400 interrupt status. |
| `bma400_interrupt_status_ready` | boolean | none | True when status read succeeded. |
| `bma400_wakeup_interrupt` | boolean | none | BMA400 wakeup interrupt bit. |
| `bma400_activity_interrupt` | boolean | none | BMA400 generic activity interrupt bit. |

## Compatibility Guidance

- API/storage should persist fields opportunistically and ignore unknown fields.
- Website/UI should not require these fields until enough devices have updated.
- Derived analytics should treat missing quality fields as `unknown`, not `bad`.
- `http_status`, `post_ms`, and `retry_count` describe the previous completed POST because current response metadata is not available before sending.
