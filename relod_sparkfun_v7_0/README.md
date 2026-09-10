# Relod firmware v7.0

Target: SparkFun Thing Plus ESP32-C6 with an Adafruit eInk Breakout Friend
(product 4224) and Adafruit 2.13-inch 250x122 SSD1680Z panel (product 6383).

## E-ink wiring

| Breakout Friend | ESP32-C6 |
| --- | --- |
| VCC | 3.3V |
| GND | GND |
| MOSI / DIN | IO4 |
| ECS / CS | IO5 |
| D/C | IO17 |
| RST | IO16 |
| BUSY | IO18 |
| SCK / CLK | IO19 |

The firmware uses the ESP32's RAM for the framebuffer, so `SRCS` is not
assigned to a GPIO. Keep the breakout's SRAM deselected (tie `SRCS` to 3.3V
if it is not already pulled high on your board revision). MISO is not needed.

GPIO9 is an ESP32-C6 boot-mode strapping pin and must be high during reset.
Do not connect the display's `BUSY` output to GPIO9; v7.0 uses GPIO18.

If the image has an approximately 8-pixel blank/shifted strip, change
`EPD_COLSTART` in `src/main.cpp` from `0` to `8` and rebuild.

## Display behavior

The screen shows:

- Relod ID (`RELOD-` plus the last four MAC-address characters)
- Last-opened local date/time
- Connected Wi-Fi SSID
- Wi-Fi signal-strength and battery state-of-charge icons
- Temperature and humidity from the Si7021
- Predicted depletion date placeholder (`MM-DD-YYYY`)
- Demo contents: `Tantalizing Turkish`

A GPIO/motion wake or first power-on records a new opened timestamp after NTP
sync. Scheduled timer wakes preserve the previous timestamp. The default time
zone is Pacific time with daylight saving; edit `TIME_ZONE` in `src/main.cpp`
if the Relod will be deployed elsewhere.

## Build and upload in VS Code

Open this folder itself in VS Code:

`C:\Users\choro\Documents\relod_sparkfun_v7_0`

Then use PlatformIO's **Build** and **Upload** actions. The project pins the
pioarduino ESP32 platform needed for ESP32-C6 Arduino support and stores its
toolchain under `C:\pio` to avoid Windows path-length installation failures.

CLI equivalents:

```powershell
pio run -e sparkfun_c6
pio run -e sparkfun_c6 --target upload
pio device monitor --baud 115200
```
