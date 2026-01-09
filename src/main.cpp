// Optimized for XIAO ESP32C6 memory constraints (512KB SRAM, 4MB Flash)
// Conditional debug output, optimized JSON sizes, and reduced String allocations

#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <WiFiManager.h>
#include <SparkFun_VL53L5CX_Library.h>
#include "Adafruit_Si7021.h"
#include <Adafruit_Sensor.h>
#include "SparkFun_BMA400_Arduino_Library.h"
#include <SparkFun_MAX1704x_Fuel_Gauge_Arduino_Library.h>
#include "esp_sleep.h"
#include <Update.h>
#include <WiFiClientSecure.h>

#define CURRENT_FIRMWARE_VERSION "6.0"

// Debug macro - only active when DEBUG_MODE is defined
#ifdef DEBUG_MODE
  #define DEBUG_PRINT(x) Serial.print(x)
  #define DEBUG_PRINTLN(x) Serial.println(x)
  #define DEBUG_PRINTF(fmt, ...) Serial.printf(fmt, __VA_ARGS__)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
  #define DEBUG_PRINTF(fmt, ...)
#endif

// Global sensor objects
SparkFun_VL53L5CX myImager;
VL53L5CX_ResultsData measurementData;
SFE_MAX1704X lipo;
BMA400 accelerometer;

uint8_t i2cAddress = BMA400_I2C_ADDRESS_DEFAULT;
int interruptPin = 2;
#define INTERRUPT_PIN 2
volatile bool interruptOccurred = false;

const char* serverName = "https://relod.fly.dev/measurement";
long TIME_TO_SLEEP = 10800;

// Function prototypes
void setupWiFi();
void initSensors();
void processRangingData();
void sendDataToServer(float, float, float, float, float, float, float, float);
void readMacAddress(char* macStr, size_t len);
void bma400InterruptHandler();

uint8_t errorCnt = 0;
double voltage = 0;
double soc = 0;
bool alert;
int dataArray[64];
Adafruit_Si7021 sensor = Adafruit_Si7021();

// Optimized: reduced from 512 to 256 bytes based on actual payload size
// Typical response: {"version":"6.0","url":"https://..."}
bool getFirmwareUpdateUrl(char* urlBuffer, size_t bufferSize) {
  HTTPClient http;
  bool updateAvailable = false;

  http.begin("https://relod.fly.dev/latest_firmware");
  int httpCode = http.GET();

  if (httpCode == 200) {
    String payload = http.getString();
    StaticJsonDocument<256> doc;

    if (!deserializeJson(doc, payload)) {
      const char* version = doc["version"];
      if (version && strcmp(version, CURRENT_FIRMWARE_VERSION) != 0) {
        const char* url = doc["url"];
        if (url && strlen(url) < bufferSize) {
          strncpy(urlBuffer, url, bufferSize - 1);
          urlBuffer[bufferSize - 1] = '\0';
          updateAvailable = true;
        }
      }
    }
  }

  http.end();
  return updateAvailable;
}

bool performOTA(const char* firmwareUrl) {
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
  #ifdef DEBUG_MODE
    Serial.begin(115200);
    delay(1000);
  #endif

  analogReadResolution(12);
  pinMode(3, INPUT);
  pinMode(1, INPUT);
  pinMode(0, INPUT);

  Wire.begin();
  Wire.setClock(100000);

  DEBUG_PRINTF("Free heap at startup: %d bytes\n", ESP.getFreeHeap());

  initSensors();

  esp_deep_sleep_enable_gpio_wakeup(1 << INTERRUPT_PIN, ESP_GPIO_WAKEUP_GPIO_HIGH);

  WiFi.mode(WIFI_STA);
  setupWiFi();

  // Check for firmware updates
  char updateUrl[256];
  if (getFirmwareUpdateUrl(updateUrl, sizeof(updateUrl))) {
    DEBUG_PRINTLN("Firmware update available, starting OTA...");
    performOTA(updateUrl);
  }

  DEBUG_PRINTF("Free heap after setup: %d bytes\n", ESP.getFreeHeap());
}

bool firstRun = true;

void loop() {
  DEBUG_PRINTF("Free heap at loop start: %d bytes\n", ESP.getFreeHeap());

  float temperature, humidity, acc_x, acc_y, acc_z;
  float voltage_pin3 = analogRead(3) * (3.3 / 4095.0);
  float voltage_pin1 = analogRead(1) * (3.3 / 4095.0);
  float voltage_pin0 = analogRead(0) * (3.3 / 4095.0);

  voltage = lipo.getVoltage();
  soc = lipo.getSOC();
  alert = lipo.getAlert();

  if (!firstRun) {
    if (!myImager.setPowerMode(SF_VL53L5CX_POWER_MODE::WAKEUP)) {
      DEBUG_PRINTLN("Failed to wake imager");
      while (1);
    }
    delay(100);
  } else {
    firstRun = false;
  }

  delay(100);
  processRangingData();
  myImager.stopRanging();
  myImager.setPowerMode(SF_VL53L5CX_POWER_MODE::SLEEP);

  humidity = sensor.readHumidity();
  temperature = sensor.readTemperature();

  accelerometer.getSensorData();
  acc_x = accelerometer.data.accelX;
  acc_y = accelerometer.data.accelY;
  acc_z = accelerometer.data.accelZ;

  if (WiFi.status() == WL_CONNECTED) {
    sendDataToServer(temperature, humidity, acc_x, acc_y, acc_z, voltage_pin0, voltage_pin3, voltage_pin1);
  } else {
    DEBUG_PRINTLN("WiFi not connected, skipping data transmission");
  }

  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);

  DEBUG_PRINTF("Free heap before sleep: %d bytes\n", ESP.getFreeHeap());

  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * 1000000ULL);
  esp_deep_sleep_start();
}

void setupWiFi() {
  WiFiManager wm;
  wm.autoConnect("relod", "password");

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
  }

  DEBUG_PRINTLN("WiFi connected");
}

void initSensors() {
  Wire.begin();
  Wire.setClock(400000);

  myImager.begin(0x29, Wire);
  myImager.setPowerMode(SF_VL53L5CX_POWER_MODE::WAKEUP);
  delay(10);
  myImager.setResolution(8 * 8);
  myImager.startRanging();

  sensor.begin();

  while (accelerometer.beginI2C(i2cAddress) != BMA400_OK) {
    DEBUG_PRINTLN("Waiting for BMA400...");
    delay(1000);
  }

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

  accelerometer.setInterruptPinMode(BMA400_INT_CHANNEL_1, BMA400_INT_PUSH_PULL_ACTIVE_1);
  accelerometer.enableInterrupt(BMA400_AUTO_WAKEUP_EN, true);
  accelerometer.enableInterrupt(BMA400_GEN2_INT_EN, true);
  attachInterrupt(digitalPinToInterrupt(interruptPin), bma400InterruptHandler, RISING);

  if (lipo.begin()) {
    lipo.quickStart();
    lipo.setThreshold(20);
    DEBUG_PRINTLN("Fuel gauge initialized");
  }
}

void processRangingData() {
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

void sendDataToServer(float temperature, float humidity, float acceleration_x,
                      float acceleration_y, float acceleration_z,
                      float voltage_red, float voltage_yellow, float voltage_green) {
  HTTPClient http;
  http.begin(serverName);
  http.addHeader("Content-Type", "application/json");

  // Optimized: reduced from 1024 to 768 bytes
  // Actual payload size is ~650-700 bytes with all fields
  StaticJsonDocument<768> jsonDoc;

  // Use char buffer instead of String for MAC address
  char macAddress[18];
  readMacAddress(macAddress, sizeof(macAddress));

  jsonDoc["device_id"] = macAddress;
  jsonDoc["temperature"] = temperature;
  jsonDoc["humidity"] = humidity;
  jsonDoc["voltage"] = voltage;
  jsonDoc["soc"] = soc;
  jsonDoc["voltage_red"] = voltage_red;
  jsonDoc["voltage_yellow"] = voltage_yellow;
  jsonDoc["voltage_green"] = voltage_green;
  jsonDoc["firmware_version"] = CURRENT_FIRMWARE_VERSION;

  JsonArray distanceArray = jsonDoc.createNestedArray("distance_mm");
  for (int i = 0; i < 64; i++) {
    distanceArray.add(dataArray[i]);
  }

  jsonDoc["acceleration_x"] = acceleration_x;
  jsonDoc["acceleration_y"] = acceleration_y;
  jsonDoc["acceleration_z"] = acceleration_z;

  String requestBody;
  serializeJson(jsonDoc, requestBody);

  DEBUG_PRINTF("Payload size: %d bytes\n", requestBody.length());
  DEBUG_PRINTF("Free heap before POST: %d bytes\n", ESP.getFreeHeap());

  int httpResponseCode = http.POST(requestBody);

  DEBUG_PRINTF("Free heap after POST: %d bytes\n", ESP.getFreeHeap());
  DEBUG_PRINTF("HTTP Response: %d\n", httpResponseCode);

  if (httpResponseCode > 0) {
    DEBUG_PRINTLN(httpResponseCode);
    DEBUG_PRINTLN(http.getString());
  } else {
    // Retry loop with error limit
    while (httpResponseCode <= 0 && errorCnt <= 5) {
      errorCnt++;

      DEBUG_PRINTF("Error Count: %d\n", errorCnt);
      DEBUG_PRINTF("Error on sending POST: %d\n", httpResponseCode);
      DEBUG_PRINTLN(http.errorToString(httpResponseCode).c_str());
      DEBUG_PRINTF("WiFi Status: %d\n", WiFi.status());

      // Restart after too many failures to prevent memory issues
      if (errorCnt > 5) {
        DEBUG_PRINTLN("Too many errors, restarting...");
        ESP.restart();
      }

      DEBUG_PRINTLN("Retrying POST request...");
      httpResponseCode = http.POST(requestBody);

      DEBUG_PRINTF("Retry response: %d\n", httpResponseCode);
    }
  }

  delay(100);
  http.end();

  // Reset error counter on success
  if (httpResponseCode > 0) {
    errorCnt = 0;
  }
}

// Optimized: use char buffer instead of returning String object
void readMacAddress(char* macStr, size_t len) {
  if (len < 18) return; // Need at least 18 bytes for MAC address string

  uint8_t baseMac[6];
  esp_wifi_get_mac(WIFI_IF_STA, baseMac);
  snprintf(macStr, len, "%02x:%02x:%02x:%02x:%02x:%02x",
           baseMac[0], baseMac[1], baseMac[2], baseMac[3], baseMac[4], baseMac[5]);
}

void bma400InterruptHandler() {
  interruptOccurred = true;
}
