# Relod Firmware

ESP32-based IoT environmental sensor that collects data and sends it to the cloud.

## What Relod Does

1. Wakes from deep sleep every 3 hours
2. Reads sensors (distance array, temperature, humidity, battery, acceleration)
3. POSTs JSON data to `https://relod.fly.dev/measurement`
4. Returns to deep sleep

## Current Production Code

**`relod_v4_0/`** - Version 4.0 is the current production firmware.

## Hardware

- **Board:** ESP32
- **VL53L5CX:** 8x8 Time-of-Flight distance sensor (64 readings per cycle)
- **BMA400:** 3-axis accelerometer with interrupt support
- **Si7021:** Temperature and humidity sensor
- **MAX17043:** LiPo battery fuel gauge
- **3 analog inputs:** Voltage monitoring (pins 0, 1, 3)

## Key Configuration

- Sleep interval: 3 hours (`TIME_TO_SLEEP = 10800`)
- WiFi: Auto-configured via WiFiManager captive portal
- OTA updates: Checks `relod.fly.dev` for firmware updates
- Device ID: Uses MAC address as unique identifier

## Building

Use Arduino IDE or arduino-cli with ESP32 board support installed.

```bash
arduino-cli compile --fqbn esp32:esp32:esp32 relod_v4_0/
arduino-cli upload --fqbn esp32:esp32:esp32 -p /dev/ttyUSB0 relod_v4_0/
```

## Libraries

All required libraries are bundled in `libraries/`. Key ones:
- SparkFun_VL53L5CX (distance sensor)
- SparkFun_BMA400 (accelerometer)
- SparkFun_MAX1704x (battery gauge)
- Adafruit_Si7021 (temp/humidity)
- WiFiManager (WiFi setup portal)
- ArduinoJson (data serialization)

## Legacy Folders

The repo contains older development versions (`relod_1hr_sleep_*`, `sketch_*`, etc.). These are historical iterations - all current work should be in `relod_v4_0/`.

## Contributing

See [AGENTS.md](AGENTS.md) for coding style, commit conventions, PR guidelines, and build/flash commands.

## Suggested Workflow

This repo uses the **compound-engineering** plugin. Follow this workflow for new features or changes:

### 1. Plan the work
```
/workflows:plan
```
Describe the feature or change you want to make. Claude will create a detailed plan.

### 2. Review the plan (do both in parallel)

When prompted, choose **"Open in editor"**. Then immediately run:
```
/plan_review
```

While the sub-agents review the plan, read through it yourself in the editor. This is efficient because:
- The plan is usually good on its own
- The agents will catch anything that needs changing
- You get familiar with the plan before it gets enhanced with suggestions

Edit or delete sections as needed based on your reading and the agent feedback.

### 3. Implement the plan
```
/workflows:work
```
Claude will execute the plan step by step, creating the code.

### 4. Create PR and review

Once implementation is done, create a PR. Then run:
```
/workflows:review
```
This performs a thorough code review before merging.
