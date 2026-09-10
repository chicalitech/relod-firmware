# Hello World Test Program for XIAO ESP32C6

## Purpose

This minimal test program validates the VS Code + PlatformIO development environment setup for the XIAO ESP32C6. It verifies that the complete toolchain works correctly: building firmware, flashing to hardware, and communicating via serial monitor using the pioarduino platform.

## What This Test Validates

- ✅ PlatformIO builds firmware successfully for XIAO ESP32C6
- ✅ Firmware flashes to the hardware without errors
- ✅ Serial monitor communication works at 115200 baud
- ✅ pioarduino platform integration functions correctly
- ✅ Basic GPIO control (LED blinking)
- ✅ ESP32C6 system information is accessible

## Hardware Requirements

- Seeed Studio XIAO ESP32C6
- USB-C cable for programming and power

## Software Requirements

- VS Code with PlatformIO extension
- Platform: [pioarduino/platform-espressif32](https://github.com/pioarduino/platform-espressif32)

## Expected Behavior

When running correctly, the program will:

1. **On startup**: Print a welcome banner and system information including:
   - Chip model and revision
   - CPU frequency
   - Flash size
   - Free heap memory
   - SDK version

2. **During operation**:
   - Print uptime and free heap memory every second
   - Blink the onboard LED (GPIO 15) at 1Hz (500ms on, 500ms off)

## How to Build and Test

### Method 1: Using VS Code PlatformIO Extension

1. Open the `hello_world_test` folder in VS Code
2. PlatformIO should auto-detect the project
3. Click the **Build** button (checkmark icon) in the bottom toolbar
4. Click the **Upload** button (arrow icon) to flash to hardware
5. Click the **Serial Monitor** button (plug icon) to view output

### Method 2: Using Command Line

```bash
# Navigate to the test directory
cd hello_world_test

# Build the firmware
pio run

# Upload to XIAO ESP32C6
pio run --target upload

# Open serial monitor (115200 baud)
pio device monitor -b 115200
```

### Method 3: Build from Repository Root

```bash
# Build from repository root
pio run -c hello_world_test/platformio.ini

# Upload from repository root
pio run -c hello_world_test/platformio.ini --target upload

# Monitor
pio device monitor -b 115200
```

## Expected Serial Output

```
=================================
Hello World from XIAO ESP32C6!
=================================

System Information:
------------------
Chip Model:    ESP32-C6
Chip Revision: 0
CPU Frequency: 160 MHz
Flash Size:    4 MB
Free Heap:     XXXXX bytes
SDK Version:   vX.X.X
------------------

Starting main loop...
(LED should blink at 1Hz)

✓ Alive! Uptime: 1 seconds | Free Heap: XXXXX bytes
✓ Alive! Uptime: 2 seconds | Free Heap: XXXXX bytes
✓ Alive! Uptime: 3 seconds | Free Heap: XXXXX bytes
...
```

## Troubleshooting

### Build fails with "command not found: pio"

- Install PlatformIO Core CLI or use VS Code extension
- Verify PlatformIO is in your PATH

### Upload fails with "serial port not found"

- Check USB-C cable connection
- Verify XIAO ESP32C6 is powered on
- Try a different USB port
- On Windows: Check Device Manager for COM port
- On Linux: Check `ls /dev/ttyACM*` or `/dev/ttyUSB*`

### Serial monitor shows no output

- Verify baud rate is set to 115200
- Press the RESET button on the XIAO ESP32C6
- Check that `ARDUINO_USB_CDC_ON_BOOT=1` flag is set in platformio.ini

### LED doesn't blink

- GPIO 15 is the correct LED pin for XIAO ESP32C6
- If LED still doesn't work, the program may still be running correctly
- Check serial output to confirm

## Success Criteria

This test is successful when:

- [x] Code compiles without errors
- [x] Firmware uploads successfully
- [x] Serial monitor displays startup message
- [x] Counter/timestamp updates every second
- [x] Onboard LED blinks at 1Hz
- [x] System information displays correctly
- [x] No warnings or errors during build, upload, or runtime

## Next Steps

Once this test passes successfully:

1. Your development environment is correctly configured
2. You can proceed to work with the full relod firmware
3. Reference this configuration when troubleshooting build issues

## Related Documentation

- [XIAO ESP32C6 Getting Started](https://wiki.seeedstudio.com/xiao_esp32c6_getting_started/)
- [PlatformIO Documentation](https://docs.platformio.org/)
- [pioarduino Platform](https://github.com/pioarduino/platform-espressif32)

## Related Issues

- Issue #6 - Create Hello World test program
- Issue #2 - Memory optimization for XIAO ESP32C6
- Issue #1 - Migration from SparkFun to XIAO ESP32C6
