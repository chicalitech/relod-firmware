#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <WiFiManager.h>
#include <SparkFun_VL53L5CX_Library.h>
#include "Adafruit_Si7021.h"
#include <Adafruit_MMA8451.h>
#include <Adafruit_Sensor.h>
#include <SparkFun_MAX1704x_Fuel_Gauge_Arduino_Library.h>

SparkFun_VL53L5CX myImager;
VL53L5CX_ResultsData measurementData;
SFE_MAX1704X lipo;

int dataArray[64];
Adafruit_MMA8451 mma = Adafruit_MMA8451();
Adafruit_Si7021 sensor = Adafruit_Si7021();
const char* serverName = "https://relod.fly.dev/measurement";

long TIME_TO_SLEEP = 3600; // Time ESP32 will go to sleep (in seconds)

double voltage = 0; // Variable to keep track of LiPo voltage
double soc = 0; // Variable to keep track of LiPo state-of-charge (SOC)
bool alert; // Variable to keep track of whether alert has been triggered

void setupWiFi();
void initSensors();
void processRangingData();
void sendDataToServer(float temperature, float humidity, float acceleration_x, float acceleration_y, float acceleration_z);
String readMacAddress();

bool enableHeater = false;
uint8_t loopCnt = 0;
uint8_t loopCnt_1 = 0;
uint8_t errorCnt = 0;

void setup() {
  //Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  setupWiFi();

  Wire.begin();
  Wire.setClock(100000);
  
  initSensors();

  //Serial.println("Setup complete");
}

void loop() {
  static uint8_t errorCnt = 0;
  float temperature, humidity, acc_x, acc_y, acc_z;

  voltage = lipo.getVoltage();
  soc = lipo.getSOC();
  alert = lipo.getAlert();

  // Reduce or remove frequent Serial prints in production
  // Serial.printf("Voltage: %.2f V\nPercentage: %.2f %%\nAlert: %d\n\n", voltage, soc, alert);

  processRangingData();
  delay(50);
  processRangingData();

  humidity = sensor.readHumidity();
  temperature = sensor.readTemperature();
  // Serial.printf("Humidity: %.2f RH\nTemperature: %.2f °C\n\n", humidity, temperature);

  sensors_event_t event;
  mma.getEvent(&event);
  acc_x = event.acceleration.x;
  acc_y = event.acceleration.y;
  acc_z = event.acceleration.z;
  // Serial.printf("X: %.2f\tY: %.2f\tZ: %.2f\tm/s^2\n", acc_x, acc_y, acc_z);

  String device_id = readMacAddress();
  // Serial.printf("[DEFAULT] ESP32 Board MAC Address: %s\n", device_id.c_str());
  // Serial.printf("Free heap before processing: %d\n", ESP.getFreeHeap());

  if (WiFi.status() == WL_CONNECTED) {
    sendDataToServer(temperature, humidity, acc_x, acc_y, acc_z);
  } else {
    // Serial.println("WiFi not connected");
  }

  // Serial.printf("Entering deep sleep for %d seconds...\n", TIME_TO_SLEEP);
  delay(10);
  //disconnect WiFi as it's no longer needed
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);

  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * 1000000ULL);
  esp_deep_sleep_start();
}

void setupWiFi() {
  WiFiManager wm;
  bool res = wm.autoConnect("relod", "password");
  if (!res) {
    // Serial.println("Failed to connect");
  } else {
    // Serial.println("connected...yeey :)");
  }

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    // Serial.print(".");
  }
  // Serial.println("\nWiFi Connected");
}

void initSensors() {
  // Serial.println("Initializing VL53L5CX Imager board...");
  if (!myImager.begin()) {
    // Serial.println(F("Sensor not found - check your wiring. Freezing"));
    while (1);
  }
  myImager.setResolution(8*8);
  myImager.startRanging();

  // Serial.println("Si7021 test!");
  if (!sensor.begin()) {
    // Serial.println("Did not find Si7021 sensor!");
    while (true);
  }

  // Serial.printf(" Rev(%d) Serial #%08X%08X\n", sensor.getRevision(), sensor.sernum_a, sensor.sernum_b);

  // Serial.println("Adafruit MMA8451 test!");
  if (!mma.begin()) {
    // Serial.println("Couldnt start");
    while (1);
  }
  mma.setRange(MMA8451_RANGE_2_G);
  // Serial.printf("Range = %dG\n", 2 << mma.getRange());

  if (!lipo.begin()) {
    // Serial.println(F("MAX17043 not detected. Please check wiring. Freezing."));
    while (1);
  }
  lipo.quickStart();
  lipo.setThreshold(20);
}

void processRangingData() {
  int imageResolution = myImager.getResolution();
  int imageWidth = sqrt(imageResolution);
  int dataIndex = 0;

  if (myImager.isDataReady()) {
    // Serial.println("\n********************************\n");

    if (myImager.getRangingData(&measurementData)) {
      for (int y = 0; y <= imageWidth * (imageWidth - 1); y += imageWidth) {
        for (int x = imageWidth - 1; x >= 0; x--) {
          // Serial.print("\t");
          // Serial.print(measurementData.distance_mm[x + y]);
          if (dataIndex < 64) {
            dataArray[dataIndex++] = measurementData.distance_mm[x + y];
          }
        }
        // Serial.println();
      }
      // Serial.println();

      long sum = 0;
      for (int i = 0; i < dataIndex; i++) {
        sum += dataArray[i];
      }

      float average = (dataIndex > 0) ? sum / (float)dataIndex : 0;
      // Serial.printf("Average distance: %.2f\n", average);

      // for (int i = 0; i < dataIndex; i++) {
      //   Serial.println(dataArray[i]);
      // }
    }
  }
}

void sendDataToServer(float temperature, float humidity, float acceleration_x, float acceleration_y, float acceleration_z) {
  HTTPClient http;
  // Serial.println("WiFi Connected");
  http.begin(serverName);
  http.addHeader("Content-Type", "application/json");

  StaticJsonDocument<1024> jsonDoc;
  jsonDoc["device_id"] = readMacAddress();
  jsonDoc["temperature"] = temperature;
  jsonDoc["humidity"] = humidity;
  jsonDoc["voltage"] = voltage;
  jsonDoc["soc"] = soc;

  JsonArray distanceArray = jsonDoc.createNestedArray("distance_mm");
  for (int i = 0; i < 64; i++) {
    distanceArray.add(dataArray[i]);
  }

  jsonDoc["acceleration_x"] = acceleration_x;
  jsonDoc["acceleration_y"] = acceleration_y;
  jsonDoc["acceleration_z"] = acceleration_z;

  String requestBody;
  serializeJson(jsonDoc, requestBody);

  // Serial.printf("Request body: %s\nFree heap before POST: %d\n", requestBody.c_str(), ESP.getFreeHeap());
  int httpResponseCode = http.POST(requestBody);
  // Serial.printf("Free heap after POST: %d\n", ESP.getFreeHeap());

  if (httpResponseCode > 0) {
    // Serial.println(httpResponseCode);
    // Serial.println(http.getString());
  } else {
    errorCnt++;
    // Serial.printf("Error Count: %d\nError on sending POST: %d\n%s\nWIFI Status: %d\n", errorCnt, httpResponseCode, http.errorToString(httpResponseCode).c_str(), WL_CONNECTED);
    if (errorCnt > 3) {
      ESP.restart();
    }
    // Serial.println("Error on sending POST: ");
    // Serial.println(httpResponseCode);
    // Serial.println(http.errorToString(httpResponseCode).c_str()); // Print the error string
    // Serial.println("WIFI Status: ");
    // Serial.println(WL_CONNECTED);
    delay(100);
    // Serial.println("Trying to send POST again:");
    int httpResponseCode = http.POST(requestBody);
    // Serial.println(httpResponseCode);
    // Serial.println(http.getString());
  }

  delay(100);
  http.end();
}

String readMacAddress() {
  uint8_t baseMac[6];
  esp_err_t ret = esp_wifi_get_mac(WIFI_IF_STA, baseMac);
  if (ret == ESP_OK) {
    char macStr[18];
    snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x", baseMac[0], baseMac[1], baseMac[2], baseMac[3], baseMac[4], baseMac[5]);
    return String(macStr);
  } else {
    // Serial.println("Failed to read MAC address");
    return String();
  }
}
