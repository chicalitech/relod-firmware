/*
  Relod - standalone Arduino IDE hardware test (not production firmware)

  Board: SparkFun ESP32-C6 Thing Plus
  USB CDC On Boot: Enabled. Serial Monitor: 115200 baud.
  Library Manager: install Adafruit EPD and its dependencies.

  Adafruit 4224 driver + 6383 2.13-inch SSD1680 display:
  VCC=3.3V, GND=GND, DIN=4, ECS=5, DC=17, RST=16, BUSY=18, CLK=19.
  External SRAM is not used. Keep the driver's SRAM CS (SRCS) inactive/high.
  BUSY uses GPIO18. Do not use GPIO9: a LOW level there during reset
  selects the ROM downloader rather than running the sketch.

  No sensor initialization, measurement POST, OTA, or deep sleep.
  Last-opened time and battery are deliberately shown as unavailable.
  E-ink refreshes once per boot, not on every heartbeat.
*/

#include <Arduino.h>
#include <WiFi.h>
#include <SPI.h>
#include <Adafruit_ThinkInk.h>

// Optional: fill these in to test connection to your Wi-Fi network.
// If left empty, the sketch scans Wi-Fi and still tests the display.
const char WIFI_SSID[] = "";
const char WIFI_PASSWORD[] = "";

constexpr int EPD_MOSI = 4;
constexpr int EPD_CS = 5;
constexpr int EPD_DC = 17;
constexpr int EPD_RESET = 16;
constexpr int EPD_BUSY = 18;
constexpr int EPD_SCK = 19;

ThinkInk_213_Grayscale4_MFGN display(
    EPD_DC, EPD_RESET, EPD_CS, -1, EPD_BUSY, &SPI);

String deviceId;
String wifiLabel = "Not connected";
int batteryPercent = -1; // The full firmware will replace this with MAX1704x SOC.
float temperatureC = NAN; // The full firmware will replace this with Si7021 data.
float humidityPercent = NAN;
String predictedDepletionDate = "MM-DD-YYYY";

void drawWifiStatus(int x, int baseline) {
  if (WiFi.status() != WL_CONNECTED) {
    display.drawLine(x + 3, baseline - 12, x + 15, baseline, EPD_WHITE);
    display.drawLine(x + 15, baseline - 12, x + 3, baseline, EPD_WHITE);
    return;
  }

  const int rssi = WiFi.RSSI();
  int bars = 1;
  if (rssi >= -80) bars = 2;
  if (rssi >= -67) bars = 3;
  if (rssi >= -55) bars = 4;

  for (int i = 0; i < 4; ++i) {
    const int barHeight = 4 + i * 3;
    if (i < bars) {
      display.fillRect(x + i * 5, baseline - barHeight, 4, barHeight,
                       EPD_WHITE);
    } else {
      display.drawRect(x + i * 5, baseline - barHeight, 4, barHeight,
                       EPD_WHITE);
    }
  }
}

void drawBatteryStatus(int x, int y, int percent) {
  constexpr int bodyWidth = 29;
  constexpr int bodyHeight = 14;
  display.drawRoundRect(x, y, bodyWidth, bodyHeight, 3, EPD_WHITE);
  display.fillRect(x + bodyWidth, y + 4, 3, 6, EPD_WHITE);

  if (percent < 0) {
    display.setTextColor(EPD_WHITE);
    display.setTextSize(1);
    display.setCursor(x + 11, y + 3);
    display.print('?');
    return;
  }

  const int fillWidth = map(constrain(percent, 0, 100), 0, 100, 0,
                            bodyWidth - 4);
  display.fillRect(x + 2, y + 2, fillWidth, bodyHeight - 4, EPD_WHITE);
}

void drawStatus() {
  display.clearBuffer();
  display.fillScreen(EPD_WHITE);
  display.setTextWrap(false);
  display.fillRect(0, 0, display.width(), 22, EPD_BLACK);
  display.setTextColor(EPD_WHITE);
  display.setTextSize(2);
  display.setCursor(6, 3);
  display.print(deviceId);
  drawWifiStatus(177, 18);
  drawBatteryStatus(213, 4, batteryPercent);

  display.setTextColor(EPD_BLACK);
  display.setTextSize(1);
  display.setCursor(6, 27);
  display.print("Opened: Not recorded");
  display.setCursor(6, 39);
  display.print("WiFi: ");
  display.print(wifiLabel.substring(0, 30));
  display.setCursor(6, 51);
  display.print("Temp: ");
  if (isnan(temperatureC)) {
    display.print("--.- C");
  } else {
    display.print(temperatureC, 1);
    display.print(" C");
  }
  display.print("  Humidity: ");
  if (isnan(humidityPercent)) {
    display.print("--.-%");
  } else {
    display.print(humidityPercent, 1);
    display.print('%');
  }
  display.setCursor(6, 63);
  display.print("Predicted depletion date ");
  display.print(predictedDepletionDate);
  display.drawFastHLine(6, 75, display.width() - 12, EPD_BLACK);
  display.setCursor(6, 79);
  display.print("DEMO CONTENTS");
  display.setTextSize(2);
  display.setCursor(6, 91);
  display.print("Tantalizing Turkish");

  Serial.println("Refreshing e-ink; allow several seconds...");
  Serial.flush();
  display.display();
  Serial.println("E-ink refresh returned.");
}

void setup() {
  Serial.begin(115200);
  // Wait briefly for the monitor, but do not require it to be connected.
  const unsigned long started = millis();
  while (!Serial && millis() - started < 5000) {
    delay(10);
  }
  delay(500);
  Serial.println("\n=== RELOD ARDUINO TEST ===");

  WiFi.mode(WIFI_STA);
  String mac = WiFi.macAddress();
  mac.replace(":", "");
  deviceId = "RELOD-" + mac.substring(mac.length() - 4);
  Serial.println("Device: " + deviceId);

  if (WIFI_SSID[0] != '\0') {
    Serial.println("Connecting to WiFi (15-second timeout)...");
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    const unsigned long wifiStarted = millis();
    while (WiFi.status() != WL_CONNECTED &&
           millis() - wifiStarted < 15000) {
      delay(500);
      Serial.print('.');
    }
    Serial.println();
    if (WiFi.status() == WL_CONNECTED) {
      wifiLabel = WiFi.SSID();
      Serial.print("Connected. IP: ");
      Serial.println(WiFi.localIP());
    } else {
      Serial.println("WiFi failed; continuing with the display test.");
    }
  } else {
    Serial.println("No WiFi credentials set. Scanning instead...");
    const int count = WiFi.scanNetworks();
    Serial.printf("WiFi scan result: %d networks (-1/-2 indicates failure).\n", count);
    WiFi.scanDelete();
    wifiLabel = "Credentials not set";
  }

  Serial.println("Initializing e-ink with BUSY on GPIO18...");
  Serial.flush();
  SPI.begin(EPD_SCK, -1, EPD_MOSI, EPD_CS);
  display.begin(THINKINK_MONO);
  display.setRotation(2); // 180 degrees from rotation 0
  drawStatus();
  Serial.println("Test complete. Staying awake for serial monitoring.");
}

void loop() {
  Serial.printf("Alive: %lu seconds | WiFi: %s\n", millis() / 1000,
                WiFi.status() == WL_CONNECTED ? "connected" : "not connected");
  delay(2000);
}
