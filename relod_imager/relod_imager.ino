/*
  Read an 8x8 array of distances from the VL53L5CX
  By: Nathan Seidle
  SparkFun Electronics
  Date: October 26, 2021
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  This example shows how to read all 64 distance readings at once.

  Feel like supporting our work? Buy a board from SparkFun!
  https://www.sparkfun.com/products/18642

*/
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <Wire.h> //Needed for I2C
#include <WiFiManager.h> // https://github.com/tzapu/WiFiManager
#include <SparkFun_VL53L5CX_Library.h> //http://librarymanager/All#SparkFun_VL53L5CX
#include "Adafruit_Si7021.h"
#include <Adafruit_MMA8451.h>
#include <Adafruit_Sensor.h>
#include <SparkFun_MAX1704x_Fuel_Gauge_Arduino_Library.h>

SparkFun_VL53L5CX myImager;
VL53L5CX_ResultsData measurementData; // Result data class structure, 1356 byes of RAM
SFE_MAX1704X lipo; // Defaults to the MAX17043

int dataArray[64]; // Example size, replace with actual size
int dataIndex = 0;
int imageResolution = 0; //Used to pretty print output
int imageWidth = 0; //Used to pretty print output

bool enableHeater = false;
uint8_t loopCnt = 0;
uint8_t loopCnt_1 = 0;
uint8_t errorCnt = 0;

double soc = 0; // Variable to keep track of LiPo state-of-charge (SOC)
bool alert; // Variable to keep track of whether alert has been triggered

Adafruit_MMA8451 mma = Adafruit_MMA8451();
Adafruit_Si7021 sensor = Adafruit_Si7021();

const char* serverName = "https://relod.fly.dev/measurement";

void setup()
{
  WiFi.mode(WIFI_STA); // explicitly set mode, esp defaults to STA+AP
  Serial.begin(115200);

  WiFiManager wm;
  bool res;
    // res = wm.autoConnect(); // auto generated AP name from chipid
    // res = wm.autoConnect("AutoConnectAP"); // anonymous ap
  res = wm.autoConnect("relod","password"); // password protected ap

  if(!res) {
      Serial.println("Failed to connect");
      // ESP.restart();
  } 
  else {
      //if you get here you have connected to the WiFi    
      Serial.println("connected...yeey :)");
  }

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi Connected");
  Serial.println("SparkFun VL53L5CX Imager Example");

  Wire.begin(); //This resets to 100kHz I2C
  Wire.setClock(100000); //Sensor has max I2C freq of 400kHz 
  
  Serial.println("Initializing VL53L5CX Imager board. This can take up to 10s. Please wait."); //Initalizing Imager
  if (myImager.begin() == false)
  {
    Serial.println(F("Sensor not found - check your wiring. Freezing"));
    while (1) ;
  }  
  myImager.setResolution(8*8); //Enable all 64 pads
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

  // Set up the MAX17043 LiPo fuel gauge:
  if (lipo.begin() == false) // Connect to the MAX17043 using the default wire port
  {
    Serial.println(F("MAX17043 not detected. Please check wiring. Freezing."));
    while (1);
  }

	// Quick start restarts the MAX17043 in hopes of getting a more accurate
	// guess for the SOC.
	lipo.quickStart();

	// We can set an interrupt to alert when the battery SoC gets too low.
	// We can alert at anywhere between 1% - 32%:
	lipo.setThreshold(20); // Set alert threshold to 20%.
}

void loop()
{
  float empty;
  float fullPercentage;
  float temperature;
  float humidity;
  float acceleration_x;
  float acceleration_y;
  float acceleration_z;

  // lipo.getVoltage() returns a voltage value (e.g. 3.93)
  voltage = lipo.getVoltage();
  // lipo.getSOC() returns the estimated state of charge (e.g. 79%)
  soc = lipo.getSOC();
  // lipo.getAlert() returns a 0 or 1 (0=alert not triggered)
  alert = lipo.getAlert();

  // Print the variables:
  Serial.print("Voltage: ");
  Serial.print(voltage);  // Print the battery voltage
  Serial.println(" V");

  Serial.print("Percentage: ");
  Serial.print(soc); // Print the battery state of charge
  Serial.println(" %");

  Serial.print("Alert: ");
  Serial.println(alert);
  Serial.println();

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
  
  // Send POST request with all the data to API
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    Serial.println("WiFi Connected");
    // Specify the URL
    http.begin(serverName);

    // Specify the content type
    http.addHeader("Content-Type", "application/json");

    // Create JSON object
    StaticJsonDocument<1024> jsonDoc; // Increase the size of the buffer
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

    // Serialize JSON object to string
    String requestBody;
    serializeJson(jsonDoc, requestBody);

    // Print the request body to debug
    Serial.print("Request body: ");
    Serial.println(requestBody);

    // Print free heap memory before sending the request
    Serial.print("Free heap before POST: ");
    Serial.println(ESP.getFreeHeap());

    // Send POST request
    int httpResponseCode = http.POST(requestBody);

    // Print free heap memory after sending the request
    Serial.print("Free heap after POST: ");
    Serial.println(ESP.getFreeHeap());

    // Print response
    if (httpResponseCode > 0) {
      String response = http.getString();
      Serial.println(httpResponseCode);
      Serial.println(response);
    } else {
      errorCnt++;
      Serial.println("Error Count: ");
      Serial.println(errorCnt);
      if (errorCnt > 3){
        ESP.restart();
      }
      Serial.println("Error on sending POST: ");
      Serial.println(httpResponseCode);
      Serial.println(http.errorToString(httpResponseCode).c_str()); // Print the error string
      Serial.println("WIFI Status: ");
      Serial.println(WL_CONNECTED);
      delay(100);
      WiFi.reconnect();
    }

    // End the HTTP connection
    delay(100);
    http.end();
    // Serial.print("Free heap after HTTP end: ");
    // Serial.println(ESP.getFreeHeap());

  } else {
    Serial.println("WiFi not connected");
  }

  // Print free heap memory
  // Serial.print("Free heap: ");
  // Serial.println(ESP.getFreeHeap());

  delay(180000); // Small delay between polling in ms
 
  // Print the count
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

  imageResolution = myImager.getResolution(); //Query sensor for current resolution - either 4x4 or 8x8
  imageWidth = sqrt(imageResolution); //Calculate printing width
  int dataIndex = 0;

  if (myImager.isDataReady() == true) {
    Serial.println("\n");
    Serial.print("********************************");
    Serial.println("\n");

    if (myImager.getRangingData(&measurementData)) { // Read distance data into array
      // The ST library returns the data transposed from zone mapping shown in datasheet
      // Pretty-print data with increasing y, decreasing x to reflect reality
      for (int y = 0; y <= imageWidth * (imageWidth - 1); y += imageWidth) {
        for (int x = imageWidth - 1; x >= 0; x--) {
          Serial.print("\t");
          Serial.print(measurementData.distance_mm[x + y]);

          // Check if the dataIndex is within bounds
          if (dataIndex < 64) {
            dataArray[dataIndex] = measurementData.distance_mm[x + y];
            dataIndex++;
          }
        }
        Serial.println();
      }
      Serial.println();

      // Calculate the average of dataArray
      long sum = 0; // Use long to avoid overflow
      for (int i = 0; i < dataIndex; i++) {
        sum += dataArray[i];
      }

      float average = 0;
      if (dataIndex > 0) {
        average = sum / (float)dataIndex; // Calculate average
      }

      // Print the average
      Serial.print("Average distance: ");
      Serial.println(average);

      for (int i = 0; i < dataIndex; i++) {
        Serial.println(dataArray[i]);
      }

      // float current = 0;
      // //Subtract empty value - current value
      // current = empty - average; 
      // Serial.print("Current distance: ");
      // Serial.println(current);

      // float emptyPercentage = ((average * 100) / empty);
      // fullPercentage = (100 - emptyPercentage);
      // Serial.print("Full level: ");
      // Serial.print(fullPercentage);
      // Serial.println("%");
    }
  }
}


