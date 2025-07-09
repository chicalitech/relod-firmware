#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <WiFiManager.h>
#include <SparkFun_VL53L5CX_Library.h>
#include "Adafruit_Si7021.h"
#include <Adafruit_MMA8451.h>
#include <Adafruit_Sensor.h>
#include <esp_wifi.h>
#include <esp_task_wdt.h>

SparkFun_VL53L5CX myImager;
VL53L5CX_ResultsData measurementData;
int dataArray[64];
int dataIndex = 0;
int imageResolution = 0;
int imageWidth = 0;

bool enableHeater = false;
uint8_t loopCnt = 0;
uint8_t loopCnt_1 = 0;

Adafruit_MMA8451 mma = Adafruit_MMA8451();
Adafruit_Si7021 sensor = Adafruit_Si7021();

const char* serverName = "https://relod.fly.dev/measurement";

void setup() {
  WiFi.mode(WIFI_STA);
  Serial.begin(115200);

  WiFiManager wm;
  bool res = wm.autoConnect("relod","password");

  if(!res) {
    Serial.println("Failed to connect");
  } else {
    Serial.println("connected...yeey :)");
  }

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi Connected");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  Wire.begin();
  Wire.setClock(400000);
  
  Serial.println("Initializing VL53L5CX Imager board. This can take up to 10s. Please wait.");
  if (myImager.begin() == false) {
    Serial.println(F("Sensor not found - check your wiring. Freezing"));
    while (1) ;
  }
  myImager.setResolution(8*8);
  myImager.startRanging(); 

  Serial.println("Si7021 test!");
  if (!sensor.begin()) {
    Serial.println("Did not find Si7021 sensor!");
    while (true);
  }

  Serial.print("Found model ");
  switch(sensor.getModel()) {
    case SI_Engineering_Samples:
      Serial.print("SI engineering samples"); break;
    case SI_7013:
      Serial.print("Si7013"); break;
    case SI_7020:
      Serial.print("Si7020"); break;
    case SI_7021:
      Serial.print("Si7021"); break;
    case SI_UNKNOWN:
    default:
      Serial.print("Unknown");
  }
  Serial.print(" Rev(");
  Serial.print(sensor.getRevision());
  Serial.print(")");
  Serial.print(" Serial #"); Serial.print(sensor.sernum_a, HEX); Serial.println(sensor.sernum_b, HEX);

  Serial.println("Adafruit MMA8451 test!");

  if (! mma.begin()) {
    Serial.println("Couldnt start");
    while (1);
  }
  Serial.println("MMA8451 found!");
  
  mma.setRange(MMA8451_RANGE_2_G);
  Serial.print("Range = "); Serial.print(2 << mma.getRange());  
  Serial.println("G");

  // Initialize Watchdog Timer (WDT)
  esp_task_wdt_config_t wdt_config = {
      .timeout_ms = 10000, // 10 seconds
      .idle_core_mask = 0, // Watch all cores
      .trigger_panic = true // Trigger panic if WDT times out
  };
  esp_task_wdt_init(&wdt_config);
  esp_task_wdt_add(NULL); // Add current task to WDT
}

void loop() {
  float temperature;
  float humidity;
  float acceleration_x;
  float acceleration_y;
  float acceleration_z;

  // Poll sensor for new data
  processRangingData();

  // Read humidity and temperature
  Serial.print("Humidity: ");
  humidity = sensor.readHumidity();
  Serial.print(humidity, 2);
  Serial.println(" RH");

  temperature = sensor.readTemperature();
  Serial.print("Temperature: ");
  Serial.print(temperature, 2);
  Serial.println(" °C");
  Serial.println("\n");

  // Get accelerometer event
  sensors_event_t event; 
  mma.getEvent(&event);

  acceleration_x = event.acceleration.x;
  Serial.print("X: \t"); Serial.print(acceleration_x); Serial.print("\t");
  acceleration_y = event.acceleration.y;
  Serial.print("Y: \t"); Serial.print(acceleration_y); Serial.print("\t");
  acceleration_z = event.acceleration.z;
  Serial.print("Z: \t"); Serial.print(acceleration_z); Serial.print("\t");
  Serial.println("m/s^2 ");

  String device_id = readMacAddress();
  Serial.print("[DEFAULT] ESP32 Board MAC Address: ");
  Serial.println(device_id);

  Serial.print("Free heap before processing: ");
  Serial.println(ESP.getFreeHeap());

  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    Serial.println("WiFi Connected");

    http.begin(serverName, "");

    http.addHeader("Content-Type", "application/json");

    StaticJsonDocument<1024> jsonDoc;
    jsonDoc["device_id"] = device_id;
    jsonDoc["temperature"] = temperature;
    jsonDoc["humidity"] = humidity;

    JsonArray distanceArray = jsonDoc.createNestedArray("distance_mm");
    for (int i = 0; i < 64; i++) {
      distanceArray.add(dataArray[i]);
    }

    jsonDoc["acceleration_x"] = acceleration_x;
    jsonDoc["acceleration_y"] = acceleration_y;
    jsonDoc["acceleration_z"] = acceleration_z;

    String requestBody;
    serializeJson(jsonDoc, requestBody);

    Serial.print("Request body: ");
    Serial.println(requestBody);

    Serial.print("Free heap before POST: ");
    Serial.println(ESP.getFreeHeap());

    int httpResponseCode = http.POST(requestBody);

    Serial.print("Free heap after POST: ");
    Serial.println(ESP.getFreeHeap());

    if (httpResponseCode > 0) {
      String response = http.getString();
      Serial.println(httpResponseCode);
      Serial.println(response);
    } else {
      Serial.print("Error on sending POST: ");
      Serial.println(httpResponseCode);
      Serial.println(http.errorToString(httpResponseCode).c_str());
      Serial.println("WIFI Status: ");
      Serial.println(WL_CONNECTED);
      delay(10);
    }

    http.end();

    Serial.print("Free heap after HTTP end: ");
    Serial.println(ESP.getFreeHeap());

    // Reset WDT
    esp_task_wdt_reset();
  } else {
    Serial.println("WiFi not connected, attempting to reconnect...");
    WiFi.reconnect();
    int retry_count = 0;
    while (WiFi.status() != WL_CONNECTED && retry_count < 10) {
      delay(1000);
      Serial.print(".");
      retry_count++;
    }
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("Reconnected to WiFi");
    } else {
      Serial.println("Failed to reconnect to WiFi");
    }
  }

  Serial.print("Free heap at end of loop: ");
  Serial.println(ESP.getFreeHeap());

  delay(10000);
  Serial.print("Loop Count: ");
  Serial.println(loopCnt_1);

  loopCnt_1++;
}

String readMacAddress(){
  uint8_t baseMac[6];
  esp_err_t ret = esp_wifi_get_mac(WIFI_IF_STA, baseMac);
  if (ret == ESP_OK) {
    char macStr[18];
    snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
             baseMac[0], baseMac[1], baseMac[2],
             baseMac[3], baseMac[4], baseMac[5]);

    // Print the MAC address to the Serial monitor
    Serial.println(macStr);

    // Return the MAC address as a string
    return String(macStr);
  } else {
    Serial.println("Failed to read MAC address");
    return String();
  }
}

void processRangingData() {
  if (myImager.isDataReady()) {
    myImager.getRangingData(&measurementData);
    for (int x = 0; x < 8; x++) {
      for (int y = 0; y < 8; y++) {
        int i = x + (y * 8);
        dataArray[dataIndex++] = measurementData.distance_mm[i];
        if (dataIndex == 64) {
          dataIndex = 0;
        }
        Serial.print(measurementData.distance_mm[i]);
        Serial.print("\t");
      }
      Serial.println();
    }
    Serial.println();
  }
}
