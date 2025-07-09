// This version removes all Serial prints for production deployment and power efficiency
// Additional modifications for power saving are included where applicable

#include <WiFi.h>
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
#include "vl53l5cx_plugin_xtalk.h"
#include <Preferences.h>
#include "vl53l5cx_buffers.h"
#include <Update.h>
#include <WiFiClientSecure.h>

#define CURRENT_FIRMWARE_VERSION "4.0"

static constexpr size_t XTALK_DATA_SIZE = VL53L5CX_XTALK_BUFFER_SIZE;
uint8_t xtalkData[XTALK_DATA_SIZE];

static constexpr uint16_t XTALK_DISTANCE_MM = 600;
static constexpr uint8_t XTALK_SAMPLES = 4;
static constexpr uint8_t XTALK_REFLECTANCE = 18;

Preferences prefs;
bool hasXtalk = false;

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

void setupWiFi();
void initSensors();
void processRangingData();
void sendDataToServer(float, float, float, float, float, float, float, float);
String readMacAddress();
void bma400InterruptHandler();

uint8_t errorCnt = 0;
double voltage = 0;
double soc = 0;
bool alert;
int dataArray[64];
Adafruit_Si7021 sensor = Adafruit_Si7021();

String getFirmwareUpdateUrl() {
  HTTPClient http;
  String firmwareUrl = "";
  http.begin("https://relod.fly.dev/latest_firmware");
  int httpCode = http.GET();
  if (httpCode == 200) {
    String payload = http.getString();
    StaticJsonDocument<512> doc;
    if (!deserializeJson(doc, payload)) {
      if (String(doc["version"]) != CURRENT_FIRMWARE_VERSION) {
        firmwareUrl = String(doc["url"]);
      }
    }
  }
  http.end();
  return firmwareUrl;
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
  analogReadResolution(12);
  pinMode(3, INPUT);
  pinMode(1, INPUT);
  pinMode(0, INPUT);

  Wire.begin();
  Wire.setClock(100000);
  initSensors();

  esp_deep_sleep_enable_gpio_wakeup(1 << INTERRUPT_PIN, ESP_GPIO_WAKEUP_GPIO_HIGH);

  WiFi.mode(WIFI_STA);
  setupWiFi();

  String updateUrl = getFirmwareUpdateUrl();
  if (updateUrl != "") {
    performOTA(updateUrl);
  }
}

bool firstRun = true;

void loop() {
  float temperature, humidity, acc_x, acc_y, acc_z;
  float voltage_pin3 = analogRead(3) * (3.3 / 4095.0);
  float voltage_pin1 = analogRead(1) * (3.3 / 4095.0);
  float voltage_pin0 = analogRead(0) * (3.3 / 4095.0);

  voltage = lipo.getVoltage();
  soc = lipo.getSOC();
  alert = lipo.getAlert();

  if (!firstRun) {
    if (!myImager.setPowerMode(SF_VL53L5CX_POWER_MODE::WAKEUP)) {
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
  }

  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);

  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * 1000000ULL);
  esp_deep_sleep_start();
}

void setupWiFi() {
  WiFiManager wm;
  wm.autoConnect("relod", "password");
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
  }
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

void sendDataToServer(float temperature, float humidity, float acceleration_x, float acceleration_y, float acceleration_z, float voltage_red, float voltage_yellow, float voltage_green) {
  HTTPClient http;
  http.begin(serverName);
  http.addHeader("Content-Type", "application/json");

  StaticJsonDocument<1024> jsonDoc;
  jsonDoc["device_id"] = readMacAddress();
  jsonDoc["temperature"] = temperature;
  jsonDoc["humidity"] = humidity;
  jsonDoc["voltage"] = voltage;
  jsonDoc["soc"] = soc;
  jsonDoc["voltage_red"] = voltage_red;
  jsonDoc["voltage_yellow"] = voltage_yellow;
  jsonDoc["voltage_green"] = voltage_green;

  JsonArray distanceArray = jsonDoc.createNestedArray("distance_mm");
  for (int i = 0; i < 64; i++) {
    distanceArray.add(dataArray[i]);
  }

  jsonDoc["acceleration_x"] = acceleration_x;
  jsonDoc["acceleration_y"] = acceleration_y;
  jsonDoc["acceleration_z"] = acceleration_z;

  String requestBody;
  serializeJson(jsonDoc, requestBody);

  Serial.printf("Request body: %s\nFree heap before POST: %d\n", requestBody.c_str(), ESP.getFreeHeap());
  int httpResponseCode = http.POST(requestBody);
  Serial.printf("Free heap after POST: %d\n", ESP.getFreeHeap());

  if (httpResponseCode > 0) {
    Serial.println(httpResponseCode);
    Serial.println(http.getString());
  } else {
      while (httpResponseCode <= 0) { // Loop until the POST request is successful
          errorCnt++;
          Serial.printf("Error Count: %d\nError on sending POST: %d\n%s\nWIFI Status: %d\n", errorCnt, httpResponseCode, http.errorToString(httpResponseCode).c_str(), WL_CONNECTED);

          // Optionally, restart the device after a certain number of failures
          if (errorCnt > 5) {
              ESP.restart();
          }

          Serial.println("Error on sending POST: ");
          Serial.println(httpResponseCode);
          Serial.println(http.errorToString(httpResponseCode).c_str()); // Print the error string
          Serial.println("WIFI Status: ");
          Serial.println(WL_CONNECTED);
          
          // delay(100);
          Serial.println("Trying to send POST again:");
          httpResponseCode = http.POST(requestBody); // Retry the POST request
          Serial.println(httpResponseCode);
          Serial.println(http.getString());
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
