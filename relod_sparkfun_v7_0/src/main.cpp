// Relod firmware v7.0: sensor reporting, OTA, deep sleep, and e-ink status.

#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <SPI.h>
#include <WiFiManager.h>
#include <SparkFun_VL53L5CX_Library.h>
#include "Adafruit_Si7021.h"
#include <Adafruit_Sensor.h>
#include "SparkFun_BMA400_Arduino_Library.h"
#include <SparkFun_MAX1704x_Fuel_Gauge_Arduino_Library.h>
#include "esp_sleep.h"
#include "vl53l5cx_plugin_xtalk.h"
#include <Preferences.h>
#include "vl53l5cx_buffers.h"
#include <Update.h>
#include <WiFiClientSecure.h>
#include <Adafruit_ThinkInk.h>
#include <time.h>

#define CURRENT_FIRMWARE_VERSION "7.0"

// Adafruit eInk Breakout Friend + 2.13-inch 250x122 SSD1680Z panel.
// The Breakout Friend's external SRAM is not wired, so the ESP32 uses RAM.
constexpr int EPD_MOSI = 4;
constexpr int EPD_CS = 5;
// GPIO9 is an ESP32-C6 boot strap, so production wiring uses GPIO18. The
// soldered-test PlatformIO environment overrides this to GPIO9 temporarily.
#ifndef EPD_BUSY_PIN
#define EPD_BUSY_PIN 18
#endif
constexpr int EPD_BUSY = EPD_BUSY_PIN;
constexpr int EPD_RESET = 16;
constexpr int EPD_DC = 17;
constexpr int EPD_SCK = 19;
constexpr int EPD_SRAM_CS = -1;
constexpr int EPD_COLSTART = 0;

constexpr char DEMO_CONTENTS[] = "Tantalizing Turkish";
constexpr char DEMO_DEPLETION_DATE[] = "MM-DD-YYYY";
constexpr char TIME_ZONE[] = "PST8PDT,M3.2.0,M11.1.0";

ThinkInk_213_Grayscale4_MFGN display(
    EPD_DC, EPD_RESET, EPD_CS, EPD_SRAM_CS, EPD_BUSY, &SPI,
    EPD_COLSTART);

// #include <FastLED.h>
// #define NUM_LEDS 1
// #define DATA_PIN 23
// CRGB leds[NUM_LEDS];

SparkFun_VL53L5CX myImager;
VL53L5CX_ResultsData measurementData;
SFE_MAX1704X lipo;
BMA400 accelerometer;

uint8_t i2cAddress = BMA400_I2C_ADDRESS_DEFAULT;
int interruptPin = 3;
#define INTERRUPT_PIN 3
volatile bool interruptOccurred = false;

const char* serverName = "https://relod.fly.dev/measurement";
long TIME_TO_SLEEP = 10800;

void setupWiFi();
void initSensors();
void processRangingData();
void sendDataToServer(float, float, float, float, float, float, float, float);
String readMacAddress();
String macLast4();
void bma400InterruptHandler();
void initDisplay();
void updateDisplay(const String&, const String&, double, float, float);
void drawWifiStatus(int, int);
void drawBatteryStatus(int, int, int);
void recordOpenedTimeIfNeeded();
String loadLastOpenedTime();
bool isNewerVersion(const char*, const char*);

uint8_t errorCnt = 0;
double voltage = 0;
double soc = 0;
bool alert;
int dataArray[64];
Adafruit_Si7021 sensor = Adafruit_Si7021();
bool imagerReady = false;
bool temperatureSensorReady = false;
bool accelerometerReady = false;
bool fuelGaugeReady = false;
bool displayReady = false;

String getFirmwareUpdateUrl() {
  HTTPClient http;
  String firmwareUrl = "";

  http.begin("https://relod.fly.dev/latest_firmware");
  int httpCode = http.GET();

  if (httpCode == 200) {
    String payload = http.getString();
    JsonDocument doc;
    if (!deserializeJson(doc, payload)) {
      const char* availableVersion = doc["version"] | "";
      if (isNewerVersion(availableVersion, CURRENT_FIRMWARE_VERSION)) {
        firmwareUrl = String(doc["url"] | "");
      }
    }
  }

  http.end();
  return firmwareUrl;
}

bool isNewerVersion(const char* candidate, const char* current) {
  int candidateParts[3] = {0, 0, 0};
  int currentParts[3] = {0, 0, 0};
  sscanf(candidate, "%d.%d.%d", &candidateParts[0], &candidateParts[1],
         &candidateParts[2]);
  sscanf(current, "%d.%d.%d", &currentParts[0], &currentParts[1],
         &currentParts[2]);

  for (int i = 0; i < 3; ++i) {
    if (candidateParts[i] != currentParts[i]) {
      return candidateParts[i] > currentParts[i];
    }
  }
  return false;
}

// Retry helper: tries POST up to maxRetries times with 2-second delay
int postWithRetries(HTTPClient &http, const String &body, int maxRetries = 2) {
  int code = -1;
  for (int i = 0; i <= maxRetries; ++i) {
    code = http.POST(body);
    if (code > 0) break; // success
    delay(2000); // short back-off
  }
  return code; // last code (or success)
}

bool performOTA(String firmwareUrl) {
  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient http;

  http.begin(client, firmwareUrl);
  int httpCode = http.GET();

  if (httpCode == 200) {
    int contentLength = http.getSize();
    if (Update.begin(contentLength)) {
      if (Update.writeStream(http.getStream()) == contentLength) {
        if (Update.end() && Update.isFinished()) {
          delay(1000);
          ESP.restart();
          return true;
        }
      }
    }
  }

  http.end();
  return false;
}

void setup() {
  Serial.begin(115200);
  // Set ADC resolution to 12 bits (default is 12, but ensure it's set before other initializations)
  delay(1500); // allow native USB serial to enumerate after reset
  Serial.println();
  Serial.println("=== RELOD boot, firmware " CURRENT_FIRMWARE_VERSION " ===");

  // FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);
  // leds[0] = CRGB::Black;
  // FastLED.show();

  analogReadResolution(12);
  pinMode(2, INPUT);
  pinMode(1, INPUT);
  pinMode(0, INPUT);

  Wire.begin();
  Wire.setClock(100000);

  Serial.println("Initializing sensors...");
  initSensors();
  Serial.println("Initializing e-ink...");
  initDisplay();

  if (fuelGaugeReady) {
    soc = lipo.getSOC();
  }
  float initialTemperature = NAN;
  float initialHumidity = NAN;
  if (temperatureSensorReady) {
    initialTemperature = sensor.readTemperature();
    initialHumidity = sensor.readHumidity();
  }

  WiFi.mode(WIFI_STA);
  updateDisplay(loadLastOpenedTime(), "Starting WiFi...",
                fuelGaugeReady ? soc : -1.0, initialTemperature,
                initialHumidity);

  esp_deep_sleep_enable_gpio_wakeup(1 << INTERRUPT_PIN, ESP_GPIO_WAKEUP_GPIO_HIGH);

  // --- Wi-Fi bring-up (one per wake cycle) ---
  // Use a strong, responsive radio while connecting or hosting the setup AP.
  // Wi-Fi is shut down before deep sleep, so this brief increase has little
  // effect on overall battery life.
  WiFi.setSleep(false);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);

  // Run WiFiManager or connect from stored creds
  Serial.println("Starting WiFi...");
  setupWiFi(); // your custom AP-name logic

  // Give it up to 10 s to connect
  if (WiFi.waitForConnectResult(10000) != WL_CONNECTED) {
    Serial.println("WiFi connection failed; updating display and sleeping.");
    if (fuelGaugeReady) {
      soc = lipo.getSOC();
    }
    updateDisplay(loadLastOpenedTime(), "Not connected",
                  fuelGaugeReady ? soc : -1.0, initialTemperature,
                  initialHumidity);

    // couldn't connect - shut Wi-Fi down and deep-sleep
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * 1000000ULL);
    esp_deep_sleep_start();
    return; // safety exit
  }

  Serial.print("WiFi connected: ");
  Serial.println(WiFi.SSID());

  recordOpenedTimeIfNeeded();

  String updateUrl = getFirmwareUpdateUrl();
  if (updateUrl != "") {
    performOTA(updateUrl);
  }
}

bool firstRun = true;

void loop() {
  float temperature = NAN;
  float humidity = NAN;
  float acc_x = NAN;
  float acc_y = NAN;
  float acc_z = NAN;
  float voltage_pin2 = analogRead(2) * (3.3 / 4095.0);
  float voltage_pin1 = analogRead(1) * (3.3 / 4095.0);
  float voltage_pin0 = analogRead(0) * (3.3 / 4095.0);

  if (fuelGaugeReady) {
    voltage = lipo.getVoltage();
    soc = lipo.getSOC();
    alert = lipo.getAlert();
  }

  if (imagerReady && !firstRun) {
    if (!myImager.setPowerMode(SF_VL53L5CX_POWER_MODE::WAKEUP)) {
      Serial.println("VL53L5CX wake failed; continuing without ranging.");
      imagerReady = false;
    }
    delay(100);
  } else {
    firstRun = false;
  }

  if (imagerReady) {
    delay(100);
    processRangingData();
    myImager.stopRanging();
    myImager.setPowerMode(SF_VL53L5CX_POWER_MODE::SLEEP);
  }

  if (temperatureSensorReady) {
    humidity = sensor.readHumidity();
    temperature = sensor.readTemperature();
  }

  if (accelerometerReady) {
    accelerometer.getSensorData();
    acc_x = accelerometer.data.accelX;
    acc_y = accelerometer.data.accelY;
    acc_z = accelerometer.data.accelZ;
  }

  if (WiFi.status() == WL_CONNECTED) {
    sendDataToServer(temperature, humidity, acc_x, acc_y, acc_z, voltage_pin0, voltage_pin2, voltage_pin1);
  }

  String wifiName = WiFi.status() == WL_CONNECTED ? WiFi.SSID() : "Not connected";
  updateDisplay(loadLastOpenedTime(), wifiName,
                fuelGaugeReady ? soc : -1.0, temperature, humidity);

  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);

  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * 1000000ULL);
  esp_deep_sleep_start();
}

void initDisplay() {
  SPI.begin(EPD_SCK, -1, EPD_MOSI, EPD_CS);
  display.begin(THINKINK_MONO);
  display.setRotation(2);
  displayReady = true;
  Serial.printf("E-ink initialized (BUSY=GPIO%d).\n", EPD_BUSY);
}

String clippedText(const String& value, size_t maximumLength) {
  if (value.length() <= maximumLength) {
    return value;
  }
  return value.substring(0, maximumLength - 3) + "...";
}

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

  const int fillWidth =
      map(constrain(percent, 0, 100), 0, 100, 0, bodyWidth - 4);
  display.fillRect(x + 2, y + 2, fillWidth, bodyHeight - 4, EPD_WHITE);
}

void updateDisplay(const String& lastOpened, const String& wifiName,
                   double batteryPercent, float temperatureC,
                   float humidityPercent) {
  if (!displayReady) {
    return;
  }

  Serial.print("Updating e-ink: ");
  Serial.println(wifiName);
  const int battery = batteryPercent < 0
                          ? -1
                          : constrain(static_cast<int>(round(batteryPercent)),
                                      0, 100);

  display.clearBuffer();
  display.fillScreen(EPD_WHITE);
  display.setTextWrap(false);

  display.fillRect(0, 0, display.width(), 22, EPD_BLACK);
  display.setTextColor(EPD_WHITE);
  display.setTextSize(2);
  display.setCursor(6, 3);
  display.print("RELOD-");
  display.print(macLast4());
  drawWifiStatus(177, 18);
  drawBatteryStatus(213, 4, battery);

  display.setTextColor(EPD_BLACK);
  display.setTextSize(1);
  display.setCursor(6, 27);
  display.print("Opened: ");
  display.print(clippedText(lastOpened, 28));

  display.setCursor(6, 39);
  display.print("WiFi: ");
  display.print(clippedText(wifiName, 32));

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
  display.print(DEMO_DEPLETION_DATE);

  display.drawFastHLine(6, 75, display.width() - 12, EPD_BLACK);
  display.setCursor(6, 79);
  display.print("CONTENTS");

  display.setTextSize(2);
  display.setCursor(6, 91);
  display.print(DEMO_CONTENTS);

  display.display();
  Serial.println("E-ink update complete.");
}

String loadLastOpenedTime() {
  Preferences preferences;
  if (!preferences.begin("relod", true)) {
    return "Not recorded";
  }

  String lastOpened = preferences.getString("last_opened", "Not recorded");
  preferences.end();
  return lastOpened;
}

void recordOpenedTimeIfNeeded() {
  // The timer is a scheduled measurement, not a physical opening event.
  if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_TIMER) {
    return;
  }

  configTzTime(TIME_ZONE, "pool.ntp.org", "time.nist.gov");
  struct tm currentTime;
  if (!getLocalTime(&currentTime, 5000)) {
    return;
  }

  char formattedTime[20];
  strftime(formattedTime, sizeof(formattedTime), "%Y-%m-%d %H:%M",
           &currentTime);

  Preferences preferences;
  if (preferences.begin("relod", false)) {
    preferences.putString("last_opened", formattedTime);
    preferences.end();
  }
}

// Returns last 4 hex characters (uppercase, no colons), e.g. "1234"
String macLast4() {
  uint8_t mac[6];
  esp_wifi_get_mac(WIFI_IF_STA, mac); // use STA MAC (device ID)
  char buf[5];
  snprintf(buf, sizeof(buf), "%02X%02X", mac[4], mac[5]); // last two bytes
  return String(buf);
}

void setupWiFi() {
  WiFi.mode(WIFI_STA); // ensure STA is initialized so we can read its MAC
  WiFiManager wm;

  String apName = "relod-" + macLast4(); // e.g., "relod-1234"
  const char* apPassword = "password"; // optional
  wm.setConnectTimeout(10);
  wm.setConfigPortalTimeout(180);
  wm.setAPCallback([](WiFiManager* manager) {
    const String portalName = manager->getConfigPortalSSID();
    Serial.println();
    Serial.print("WiFi setup portal started: ");
    Serial.println(portalName);
    Serial.println("Password: password");
    Serial.println("If the portal does not open automatically, visit 192.168.4.1");

    float portalTemperature = NAN;
    float portalHumidity = NAN;
    if (temperatureSensorReady) {
      portalTemperature = sensor.readTemperature();
      portalHumidity = sensor.readHumidity();
    }
    const double portalBattery = fuelGaugeReady ? lipo.getSOC() : -1.0;
    updateDisplay(loadLastOpenedTime(), "Setup: " + portalName,
                  portalBattery, portalTemperature, portalHumidity);
  });

  Serial.print("WiFiManager AP name: ");
  Serial.println(apName);

  // If saved creds connect: great. If not, config portal SSID = apName
  bool connected = wm.autoConnect(apName.c_str(), apPassword);

  if (!connected) {
    Serial.println("WiFiManager timed out without a connection.");
  }
}

void initSensors() {
  Wire.begin();
  Wire.setClock(400000);

  imagerReady = myImager.begin(0x29, Wire);
  if (imagerReady) {
    imagerReady = myImager.setPowerMode(SF_VL53L5CX_POWER_MODE::WAKEUP);
    delay(10);
    imagerReady = imagerReady && myImager.setResolution(8 * 8);
    imagerReady = imagerReady && myImager.startRanging();
  }
  Serial.println(imagerReady ? "VL53L5CX: ready" : "VL53L5CX: not detected");

  temperatureSensorReady = sensor.begin();
  Serial.println(temperatureSensorReady ? "Si7021: ready" : "Si7021: not detected");

  for (int attempt = 1; attempt <= 3 && !accelerometerReady; ++attempt) {
    accelerometerReady =
        accelerometer.beginI2C(i2cAddress) == BMA400_OK;
    if (!accelerometerReady) {
      Serial.printf("BMA400: attempt %d failed\n", attempt);
      delay(250);
    }
  }

  if (accelerometerReady) {
    Serial.println("BMA400: ready");
    accelerometer.setMode(BMA400_MODE_LOW_POWER);

    bma400_wakeup_conf wakeupConfig = {
    .wakeup_ref_update = BMA400_UPDATE_ONE_TIME,
    .sample_count = BMA400_SAMPLE_COUNT_1,
    .wakeup_axes_en = BMA400_AXIS_XYZ_EN,
    .int_wkup_threshold = 4,
    .int_wkup_ref_x = 0,
    .int_wkup_ref_y = 0,
    .int_wkup_ref_z = 64,
    .int_chan = BMA400_INT_CHANNEL_1
  };
    accelerometer.setWakeupInterrupt(&wakeupConfig);

    bma400_auto_lp_conf autoLPConfig = {
    .auto_low_power_trigger = BMA400_AUTO_LP_TIME_RESET_EN,
    .auto_lp_timeout_threshold = 400
  };
    accelerometer.setAutoLowPower(&autoLPConfig);

    bma400_gen_int_conf config = {
    .gen_int_thres = 5,
    .gen_int_dur = 1,
    .axes_sel = BMA400_AXIS_XYZ_EN,
    .data_src = BMA400_DATA_SRC_ACCEL_FILT_2,
    .criterion_sel = BMA400_ACTIVITY_INT,
    .evaluate_axes = BMA400_ANY_AXES_INT,
    .ref_update = BMA400_UPDATE_EVERY_TIME,
    .hysteresis = BMA400_HYST_48_MG,
    .int_thres_ref_x = 0,
    .int_thres_ref_y = 0,
    .int_thres_ref_z = 512,
    .int_chan = BMA400_UNMAP_INT_PIN
  };
    accelerometer.setGeneric2Interrupt(&config);

    accelerometer.setInterruptPinMode(BMA400_INT_CHANNEL_1,
                                      BMA400_INT_PUSH_PULL_ACTIVE_1);
    accelerometer.enableInterrupt(BMA400_AUTO_WAKEUP_EN, true);
    accelerometer.enableInterrupt(BMA400_GEN2_INT_EN, true);

    attachInterrupt(digitalPinToInterrupt(interruptPin),
                    bma400InterruptHandler, RISING);
  } else {
    Serial.println("BMA400: unavailable; continuing boot.");
  }

  fuelGaugeReady = lipo.begin();
  if (fuelGaugeReady) {
    lipo.quickStart();
    lipo.setThreshold(20);
  }
  Serial.println(fuelGaugeReady ? "MAX1704x: ready" : "MAX1704x: not detected");
}

void processRangingData() {
  if (!imagerReady) {
    return;
  }
  int imageResolution = myImager.getResolution();
  int imageWidth = sqrt(imageResolution);
  int dataIndex = 0;

  memset(dataArray, 0, sizeof(dataArray));

  if (myImager.isDataReady()) {
    if (myImager.getRangingData(&measurementData)) {
      for (int y = 0; y <= imageWidth * (imageWidth - 1); y += imageWidth) {
        for (int x = imageWidth - 1; x >= 0; x--) {
          if (dataIndex < 64) {
            dataArray[dataIndex++] = measurementData.distance_mm[x + y];
          }
        }
      }
    }
  }
}

void sendDataToServer(float temperature, float humidity, float acceleration_x, float acceleration_y, float acceleration_z, float voltage_red, float voltage_yellow, float voltage_green) {
  HTTPClient http;
  http.begin(serverName);
  http.addHeader("Content-Type", "application/json");

  JsonDocument jsonDoc;
  jsonDoc["device_id"] = readMacAddress();
  jsonDoc["temperature"] = temperature;
  jsonDoc["humidity"] = humidity;
  jsonDoc["voltage"] = voltage;
  jsonDoc["soc"] = soc;
  jsonDoc["voltage_red"] = voltage_red;
  jsonDoc["voltage_yellow"] = voltage_yellow;
  jsonDoc["voltage_green"] = voltage_green;
  jsonDoc["firmware_version"] = CURRENT_FIRMWARE_VERSION;

  JsonArray distanceArray = jsonDoc["distance_mm"].to<JsonArray>();
  for (int i = 0; i < 64; i++) {
    distanceArray.add(dataArray[i]);
  }

  jsonDoc["acceleration_x"] = acceleration_x;
  jsonDoc["acceleration_y"] = acceleration_y;
  jsonDoc["acceleration_z"] = acceleration_z;

  String requestBody;
  serializeJson(jsonDoc, requestBody);

  Serial.printf("Request body: %s\nFree heap before POST: %d\n", requestBody.c_str(), ESP.getFreeHeap());
  int httpResponseCode = postWithRetries(http, requestBody, 2);
  Serial.printf("Free heap after POST: %d\n", ESP.getFreeHeap());

  if (httpResponseCode > 0) {
    Serial.println(httpResponseCode);
    Serial.println(http.getString());
  } else {
    errorCnt++;
    Serial.printf("Error Count: %d\n", errorCnt);
    // Optionally reboot after repeated failures
    if (errorCnt > 5) {
      ESP.restart();
    }
  }

  delay(100);
  http.end();
}

String readMacAddress() {
  uint8_t baseMac[6];
  esp_wifi_get_mac(WIFI_IF_STA, baseMac);
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x", baseMac[0], baseMac[1], baseMac[2], baseMac[3], baseMac[4], baseMac[5]);
  return String(macStr);
}

void bma400InterruptHandler() {
  interruptOccurred = true;
}
