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
void sendDataToServer(float temperature, float humidity, float acceleration_x, float acceleration_y, float acceleration_z, float voltage_pin3, float voltage_pin0, float voltage_pin1);
String readMacAddress();

bool enableHeater = false;
uint8_t loopCnt = 0;
uint8_t loopCnt_1 = 0;
uint8_t errorCnt = 0;

void setup() {
  Serial.begin(115200);

  // Set ADC resolution to 12 bits (default is 12, but ensure it's set before other initializations)
  analogReadResolution(12);

  pinMode(3, INPUT);
  pinMode(1, INPUT);
  pinMode(0, INPUT);

  Wire.begin();
  Wire.setClock(100000);
  initSensors();

  WiFi.mode(WIFI_STA);
  setupWiFi();

  Serial.println("Setup complete");
}

bool firstRun = true; // Flag to check if it's the first run

void loop() {
  static uint8_t errorCnt = 0;
  float temperature, humidity, acc_x, acc_y, acc_z;


  // Disable Wi-Fi before reading analog input to avoid conflict
  WiFi.mode(WIFI_OFF);

  int analogValue_pin3 = analogRead(3);
  int analogValue_pin1 = analogRead(1);
  int analogValue_pin0 = analogRead(0);

  // Convert the analog value to a voltage (0 - 3.3V)
  float voltage_pin3 = analogValue_pin3 * (3.3 / 4095.0);
  float voltage_pin1 = analogValue_pin1 * (3.3 / 4095.0);
  float voltage_pin0 = analogValue_pin0 * (3.3 / 4095.0);

  // Print the voltage of the pins
  Serial.print("Voltage Red LED: ");
  Serial.println(voltage_pin3);
  Serial.print("Voltage Green LED: ");
  Serial.println(voltage_pin1);
  Serial.print("Voltage Yellow LED: ");
  Serial.println(voltage_pin0);

  // Re-enable Wi-Fi after analog read
  WiFi.mode(WIFI_STA);
  WiFi.begin();

  voltage = lipo.getVoltage();
  soc = lipo.getSOC();
  alert = lipo.getAlert();

  Serial.printf("Voltage: %.2f V\nPercentage: %.2f %%\nAlert: %d\n\n", voltage, soc, alert);

  // Skip wakeup procedure on the first run as the device is already awake
  if (!firstRun) {
    bool wakeup_response = myImager.setPowerMode(SF_VL53L5CX_POWER_MODE::WAKEUP);
    if (wakeup_response) {
      Serial.println(F("Set device to wakeup mode."));
    } else {
      Serial.println(F("Cannot wakeup device. Freezing..."));
      while (1);
    }
    delay(100); // Short delay to allow the sensor to wake up
  } else {
    firstRun = false; // Reset the flag after the first run
  }

  delay(100); // Short delay to allow the sensor to wake up
  processRangingData();

  // Attempt to stop ranging before setting sleep mode
  myImager.stopRanging();
  Serial.println("Ranging stopped.");

  // Set imager to sleep mode
  bool sleep_response = myImager.setPowerMode(SF_VL53L5CX_POWER_MODE::SLEEP);
  if (sleep_response) {
    Serial.println(F("Set device to sleep mode."));
  } else {
    Serial.println(F("Cannot set device to sleep mode. Freezing..."));
    while (1);
  }

  humidity = sensor.readHumidity();
  temperature = sensor.readTemperature();
  Serial.printf("Humidity: %.2f RH\nTemperature: %.2f °C\n\n", humidity, temperature);

  sensors_event_t event;
  mma.getEvent(&event);
  acc_x = event.acceleration.x;
  acc_y = event.acceleration.y;
  acc_z = event.acceleration.z;
  Serial.printf("X: %.2f\tY: %.2f\tZ: %.2f\tm/s^2\n", acc_x, acc_y, acc_z);

  String device_id = readMacAddress();
  Serial.printf("[DEFAULT] ESP32 Board MAC Address: %s\n", device_id.c_str());
  Serial.printf("Free heap before processing: %d\n", ESP.getFreeHeap());

  if (WiFi.status() == WL_CONNECTED) {
    sendDataToServer(temperature, humidity, acc_x, acc_y, acc_z, voltage_pin3, voltage_pin0, voltage_pin1);
  } else {
    Serial.println("WiFi not connected");
  }

  Serial.printf("Entering deep sleep for %d seconds...\n", TIME_TO_SLEEP);
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
    Serial.println("Failed to connect");
  } else {
    Serial.println("connected...yeey :)");
  }

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("\nWiFi Connected");
}

void initSensors() {

  Serial.println("Initializing VL53L5CX Imager board...");
  if (!myImager.begin()) {
    Serial.println(F("Sensor not found - check your wiring. Freezing"));
    while (1);
  }
  myImager.setResolution(8*8);
  myImager.startRanging();

  // Check if device is alive
  bool response = myImager.isConnected();
  if (response == true)
  {
    Serial.println(F("Sensor is connected."));
  }
  else
  {
    Serial.println(F("Sensor not detected. Freezing."));
    while (1) ;
  }

  Serial.println("Si7021 test!");
  if (!sensor.begin()) {
    Serial.println("Did not find Si7021 sensor!");
    while (true);
  }

  // Serial.print("Found model ");
  // switch(sensor.getModel()) {
  //   case SI_Engineering_Samples: Serial.print("SI engineering samples"); break;
  //   case SI_7013: Serial.print("Si7013"); break;
  //   case SI_7020: Serial.print("Si7020"); break;
  //   case SI_7021: Serial.print("Si7021"); break;
  //   default: Serial.print("Unknown");
  // }
  // Serial.printf(" Rev(%d) Serial #%08X%08X\n", sensor.getRevision(), sensor.sernum_a, sensor.sernum_b);

  Serial.println("Adafruit MMA8451 test!");
  if (!mma.begin()) {
    Serial.println("Couldnt start");
    while (1);
  }
  mma.setRange(MMA8451_RANGE_2_G);
  Serial.printf("Range = %dG\n", 2 << mma.getRange());

  if (!lipo.begin()) {
    Serial.println(F("MAX17043 not detected. Please check wiring. Freezing."));
    while (1);
  }
  lipo.quickStart();
  lipo.setThreshold(20);
}

void processRangingData() {
  int imageResolution = myImager.getResolution();
  int imageWidth = sqrt(imageResolution);
  int dataIndex = 0;

  // Clear the dataArray before taking a new measurement
  memset(dataArray, 0, sizeof(dataArray));

  if (myImager.isDataReady()) {
    Serial.println("\n********************************\n");

    if (myImager.getRangingData(&measurementData)) {
      for (int y = 0; y <= imageWidth * (imageWidth - 1); y += imageWidth) {
        for (int x = imageWidth - 1; x >= 0; x--) {
          Serial.print("\t");
          Serial.print(measurementData.distance_mm[x + y]);
          if (dataIndex < 64) {
            dataArray[dataIndex++] = measurementData.distance_mm[x + y];
          }
        }
        Serial.println();
      }
      Serial.println();

      long sum = 0;
      for (int i = 0; i < dataIndex; i++) {
        sum += dataArray[i];
      }

      float average = (dataIndex > 0) ? sum / (float)dataIndex : 0;
      Serial.printf("Average distance: %.2f\n", average);

      for (int i = 0; i < dataIndex; i++) {
        Serial.println(dataArray[i]);
      }
    } else {
      Serial.println("Error retrieving ranging data.");
    }
  } else {
    Serial.println("Data not ready.");
  }
}

void sendDataToServer(float temperature, float humidity, float acceleration_x, float acceleration_y, float acceleration_z, float voltage_red, float voltage_yellow, float voltage_green) {
  HTTPClient http;
  Serial.println("WiFi Connected");
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
  esp_err_t ret = esp_wifi_get_mac(WIFI_IF_STA, baseMac);
  if (ret == ESP_OK) {
    char macStr[18];
    snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x", baseMac[0], baseMac[1], baseMac[2], baseMac[3], baseMac[4], baseMac[5]);
    return String(macStr);
  } else {
    Serial.println("Failed to read MAC address");
    return String();
  }
}
