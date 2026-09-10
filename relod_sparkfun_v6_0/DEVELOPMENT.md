# ESP32 development

Open this folder in VS Code using the **ESP32 Arduino** profile. It contains
pioarduino IDE and Microsoft C/C++, with PlatformIO IDE, ESP-IDF, CMake Tools,
and Makefile Tools excluded from the profile. The separate **ESP-IDF** profile
contains Espressif's extension and the existing ESP-IDF tool paths.

To open this project explicitly from a terminal:

```powershell
code --profile "ESP32 Arduino" "C:\Users\choro\Documents\relod_sparkfun_v6_0"
```

Use **pioarduino: New Terminal** from VS Code's Command Palette for the commands
below. The profile uses the extension-managed Python and PlatformIO-compatible
core, so no global Python or PATH changes are required.

## Build environments

| Environment | Board | Logging |
| --- | --- | --- |
| `sparkfun_c6` (default) | SparkFun ESP32-C6 Thing Plus | Release, core logging off |
| `xiao_c6` | Seeed XIAO ESP32-C6 | Release, core logging off |
| `sparkfun_c6_debug` | SparkFun ESP32-C6 Thing Plus | Debug symbols, informational core logs |
| `xiao_c6_debug` | Seeed XIAO ESP32-C6 | Debug symbols, informational core logs |

```powershell
pio run -e sparkfun_c6
pio run -e xiao_c6
pio run -e sparkfun_c6_debug
pio run -e xiao_c6_debug
```

The sketch remains in its existing folder. `src_dir` in `platformio.ini` points
PlatformIO at that folder so the Arduino sketch conversion step finds it.

The Arduino SDK contains paths that exceed Windows' legacy 260-character limit
during extraction. Windows long-path support is enabled on this PC. Enable it
on other Windows machines before installing this SDK.

## Upload and monitor

The firmware exceeds the default 1.25 MiB app partition. The SparkFun board uses
`default_16MB.csv` (two 6.25 MiB OTA app slots), and the XIAO uses `min_spiffs.csv`
(two 1.875 MiB OTA app slots and a 128 KiB filesystem). Both layouts retain NVS
and OTA support. The sketch does not currently use the flash filesystem.

For the first upload after this partition-layout change, use a full PlatformIO
USB/serial upload so the new partition table is written. The sketch's existing
OTA updater only writes application firmware and cannot apply this layout change.

Connect the intended board, choose its matching environment, and use its Upload
task. For example, for the SparkFun development build:

```powershell
pio run -e sparkfun_c6_debug -t upload
pio device monitor -e sparkfun_c6_debug
```

The monitor uses 115200 baud, exception decoding, and timestamps. Monitor using
the same environment that produced the uploaded firmware; crash decoding needs
its matching local ELF file. Only one serial monitor should hold the board's
port at a time. Use `pio device list` to identify the port if autodetection is
ambiguous. If uploads are unreliable, try lowering `upload_speed` from 921600
to 460800 or 115200.

The development environments enable Arduino core messages; they do not restore
application messages removed from this production sketch. USB/JTAG breakpoint
debugging and Windows device-driver setup are separate from these logging
changes.

## Dependencies

Keep the platform release and library versions/commits pinned in `platformio.ini`.
ArduinoJson is pinned to 7.4.3, Adafruit BusIO to 1.17.4, and the six Git-based
libraries to their full commit hashes. BusIO is declared explicitly so the
Si7021 driver's indirect dependency is pinned too.
Change pins deliberately and rebuild the affected board environments before
uploading. FastLED was removed from the dependency list because its use in the
sketch is commented out.

If IntelliSense falls behind a successful build, run the pioarduino command to
rebuild the IntelliSense index. Do not hand-maintain generated SDK include paths.

## Validation

All four environments compiled successfully on 2026-09-08 with the pinned
dependencies and the partition layouts above. PlatformIO's size check reports
21.3% app-slot usage for SparkFun release and 70.1% for XIAO release; debug builds
report 23.0% and 75.5%, respectively. All generated firmware binaries also fit
their app slots. Build output is in `.pio/setup-build-validated.log`.

The existing firmware emits ArduinoJson deprecation warnings for
`StaticJsonDocument` and `createNestedArray`; these did not prevent compilation.
The platform also reports a nonfatal cleanup warning for an older cached
framework's read-only Git pack file. Uploading, sensor behavior, and hardware
debugging have not been tested as part of this setup work.

VS Code's IntelliSense configuration was regenerated for the default SparkFun
release environment. The generated launcher's executable path was corrected to
match its `sparkfun_c6_debug` environment. If a future IDE regeneration replaces
`launch.json`, ensure the executable still belongs to the selected debug
environment before using hardware debugging.
