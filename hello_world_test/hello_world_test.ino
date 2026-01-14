/*
 * Hello World Test Program for XIAO ESP32C6
 *
 * Purpose: Validate VS Code + PlatformIO development environment setup
 *
 * This minimal test program verifies:
 * - PlatformIO builds firmware successfully
 * - Firmware flashes to XIAO ESP32C6 hardware
 * - Serial monitor communication works at 115200 baud
 * - pioarduino platform integration functions correctly
 *
 * Expected behavior:
 * - Prints startup message with chip info on boot
 * - Updates uptime and heap memory every second
 * - Blinks onboard LED at 1Hz (500ms on, 500ms off)
 *
 * Hardware: Seeed Studio XIAO ESP32C6
 * Platform: pioarduino/platform-espressif32
 *
 * Related: Issue #6 - Development environment validation
 */

#define LED_BUILTIN 15  // Onboard LED on XIAO ESP32C6 is GPIO 15

void setup() {
  // Initialize serial communication at 115200 baud
  Serial.begin(115200);

  // Wait a moment for serial to initialize
  delay(100);

  // Initialize LED pin as output
  pinMode(LED_BUILTIN, OUTPUT);

  // Print startup banner
  Serial.println("\n\n=================================");
  Serial.println("Hello World from XIAO ESP32C6!");
  Serial.println("=================================\n");

  // Display chip information
  Serial.println("System Information:");
  Serial.println("------------------");
  Serial.printf("Chip Model:    %s\n", ESP.getChipModel());
  Serial.printf("Chip Revision: %d\n", ESP.getChipRevision());
  Serial.printf("CPU Frequency: %d MHz\n", ESP.getCpuFreqMHz());
  Serial.printf("Flash Size:    %d MB\n", ESP.getFlashChipSize() / (1024 * 1024));
  Serial.printf("Free Heap:     %d bytes\n", ESP.getFreeHeap());
  Serial.printf("SDK Version:   %s\n", ESP.getSdkVersion());
  Serial.println("------------------\n");

  Serial.println("Starting main loop...");
  Serial.println("(LED should blink at 1Hz)\n");
}

void loop() {
  // Calculate uptime in seconds
  unsigned long uptimeSeconds = millis() / 1000;

  // Print status update with uptime and free heap
  Serial.printf("✓ Alive! Uptime: %lu seconds | Free Heap: %d bytes\n",
                uptimeSeconds, ESP.getFreeHeap());

  // Blink LED - turn on
  digitalWrite(LED_BUILTIN, HIGH);
  delay(500);

  // Blink LED - turn off
  digitalWrite(LED_BUILTIN, LOW);
  delay(500);
}
