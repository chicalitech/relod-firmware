// Production firmware for deployed Relod containers.
// Payload changes must stay additive because older devices are already live.

#include <Arduino.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiManager.h>
#include <SparkFun_VL53L5CX_Library.h>
#include "Adafruit_Si7021.h"
#include <Adafruit_Sensor.h>
#include "SparkFun_BMA400_Arduino_Library.h"
#include <SparkFun_MAX1704x_Fuel_Gauge_Arduino_Library.h>
#include "esp_sleep.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include <Update.h>
#include <WiFiClientSecure.h>
#include "mbedtls/sha256.h"

#define CURRENT_FIRMWARE_VERSION "6.0"

namespace {

constexpr uint8_t SCHEMA_VERSION = 1;
constexpr uint64_t SLEEP_INTERVAL_S = 10800;
constexpr uint8_t INTERRUPT_PIN = 2;
constexpr uint8_t BMA400_I2C_ADDRESS = BMA400_I2C_ADDRESS_DEFAULT;
constexpr uint8_t MAX_POST_ATTEMPTS = 3;
constexpr uint32_t WIFI_CONNECT_TIMEOUT_S = 45;
constexpr uint32_t WIFI_CONFIG_PORTAL_TIMEOUT_S = 180;
constexpr uint32_t HTTP_TIMEOUT_MS = 15000;
constexpr uint32_t POST_RETRY_DELAY_MS = 750;
constexpr uint16_t DISTANCE_FAR_MM = 3500;
constexpr uint8_t DISTANCE_GRID_SIZE = 64;

const char *MEASUREMENT_URL = "https://relod.fly.dev/measurement";
const char *FIRMWARE_METADATA_URL = "https://relod.fly.dev/latest_firmware";

RTC_DATA_ATTR uint32_t bootCount = 0;
RTC_DATA_ATTR uint32_t measurementSequence = 0;
RTC_DATA_ATTR int previousHttpStatus = 0;
RTC_DATA_ATTR uint32_t previousPostMs = 0;
RTC_DATA_ATTR uint8_t previousRetryCount = 0;

SparkFun_VL53L5CX myImager;
VL53L5CX_ResultsData measurementData;
SFE_MAX1704X lipo;
BMA400 accelerometer;
Adafruit_Si7021 climateSensor = Adafruit_Si7021();

volatile bool interruptOccurred = false;
int distanceGrid[DISTANCE_GRID_SIZE];

struct WifiTelemetry {
  bool connected = false;
  uint32_t connectMs = 0;
  int32_t rssiDbm = 0;
};

struct DistanceQuality {
  bool dataReady = false;
  bool readOk = false;
  uint8_t validCount = 0;
  uint8_t zeroCount = 0;
  uint8_t farCount = 0;
  uint8_t badStatusCount = 0;
  uint16_t spreadMm = 0;
  uint8_t targetDetectedCount = 0;
  uint32_t signalAvg = 0;
  uint32_t ambientAvg = 0;
  uint8_t reflectanceAvg = 0;
};

struct MotionTelemetry {
  uint16_t interruptStatus = 0;
  bool interruptStatusReady = false;
  bool wakeupInterrupt = false;
  bool activityInterrupt = false;
};

struct FirmwareMetadata {
  String version;
  String url;
  String sha256;
  bool force = false;
};

WifiTelemetry wifiTelemetry;
DistanceQuality distanceQuality;
MotionTelemetry motionTelemetry;
esp_sleep_wakeup_cause_t wakeupCause = ESP_SLEEP_WAKEUP_UNDEFINED;
esp_reset_reason_t resetReason = ESP_RST_UNKNOWN;

void initSensors();
WifiTelemetry setupWiFi();
void processRangingData();
bool sendDataToServer(float temperature, float humidity, float accelerationX, float accelerationY, float accelerationZ,
                      float voltageRed, float voltageYellow, float voltageGreen);
String readMacAddress();
void bma400InterruptHandler();
void enterDeepSleep();
bool getFirmwareMetadata(FirmwareMetadata &metadata);
bool shouldApplyFirmwareUpdate(const FirmwareMetadata &metadata);
bool performOTA(const FirmwareMetadata &metadata);

const char *wakeupCauseToString(esp_sleep_wakeup_cause_t cause) {
  switch (cause) {
    case ESP_SLEEP_WAKEUP_TIMER:
      return "timer";
    case ESP_SLEEP_WAKEUP_GPIO:
      return "gpio";
    case ESP_SLEEP_WAKEUP_EXT0:
      return "ext0";
    case ESP_SLEEP_WAKEUP_EXT1:
      return "ext1";
    case ESP_SLEEP_WAKEUP_TOUCHPAD:
      return "touchpad";
    case ESP_SLEEP_WAKEUP_ULP:
      return "ulp";
    case ESP_SLEEP_WAKEUP_UART:
      return "uart";
    default:
      return "undefined";
  }
}

const char *resetReasonToString(esp_reset_reason_t reason) {
  switch (reason) {
    case ESP_RST_POWERON:
      return "poweron";
    case ESP_RST_EXT:
      return "external";
    case ESP_RST_SW:
      return "software";
    case ESP_RST_PANIC:
      return "panic";
    case ESP_RST_INT_WDT:
      return "interrupt_watchdog";
    case ESP_RST_TASK_WDT:
      return "task_watchdog";
    case ESP_RST_WDT:
      return "watchdog";
    case ESP_RST_DEEPSLEEP:
      return "deep_sleep";
    case ESP_RST_BROWNOUT:
      return "brownout";
    case ESP_RST_SDIO:
      return "sdio";
    default:
      return "unknown";
  }
}

bool isMotionWake(esp_sleep_wakeup_cause_t cause) {
  return interruptOccurred || cause == ESP_SLEEP_WAKEUP_GPIO || cause == ESP_SLEEP_WAKEUP_EXT0 || cause == ESP_SLEEP_WAKEUP_EXT1;
}

bool isUsableTargetStatus(uint8_t status) {
  return status == 5 || status == 9;
}

void sortUint16(uint16_t *values, uint8_t count) {
  for (uint8_t i = 1; i < count; i++) {
    uint16_t value = values[i];
    int8_t j = i - 1;
    while (j >= 0 && values[j] > value) {
      values[j + 1] = values[j];
      j--;
    }
    values[j + 1] = value;
  }
}

uint16_t percentileSpread(uint16_t *values, uint8_t count) {
  if (count < 2) {
    return 0;
  }

  sortUint16(values, count);
  uint8_t p10Index = static_cast<uint8_t>((count - 1) * 10 / 100);
  uint8_t p90Index = static_cast<uint8_t>((count - 1) * 90 / 100);
  return values[p90Index] - values[p10Index];
}

bool readVersionSegment(const String &version, int &index, long &segment) {
  segment = 0;
  bool foundDigit = false;

  while (index < version.length() && version[index] == '.') {
    index++;
  }

  while (index < version.length()) {
    char value = version[index];
    if (value == '.') {
      index++;
      break;
    }

    if (value < '0' || value > '9') {
      return false;
    }

    foundDigit = true;
    segment = (segment * 10) + (value - '0');
    index++;
  }

  return foundDigit;
}

int compareVersions(const String &candidate, const String &current) {
  int candidateIndex = 0;
  int currentIndex = 0;

  for (uint8_t i = 0; i < 4; i++) {
    long candidateSegment = 0;
    long currentSegment = 0;
    bool candidateHasSegment = readVersionSegment(candidate, candidateIndex, candidateSegment);
    bool currentHasSegment = readVersionSegment(current, currentIndex, currentSegment);

    if (!candidateHasSegment && !currentHasSegment) {
      return 0;
    }

    if (!candidateHasSegment) {
      candidateSegment = 0;
    }
    if (!currentHasSegment) {
      currentSegment = 0;
    }

    if (candidateSegment > currentSegment) {
      return 1;
    }
    if (candidateSegment < currentSegment) {
      return -1;
    }
  }

  return 0;
}

bool isSha256Hex(const String &value) {
  if (value.length() != 64) {
    return false;
  }

  for (int i = 0; i < value.length(); i++) {
    char c = value[i];
    bool isHex = (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f') || (c >= 'A' && c <= 'F');
    if (!isHex) {
      return false;
    }
  }

  return true;
}

String bytesToHex(const uint8_t *bytes, size_t length) {
  static const char hex[] = "0123456789abcdef";
  String result;
  result.reserve(length * 2);

  for (size_t i = 0; i < length; i++) {
    result += hex[(bytes[i] >> 4) & 0x0F];
    result += hex[bytes[i] & 0x0F];
  }

  return result;
}

String normalizedSha256(const String &value) {
  String normalized = value;
  normalized.toLowerCase();
  return normalized;
}

bool writeFirmwareStreamWithHash(WiFiClient *stream, int contentLength, const String &expectedSha256) {
  if (contentLength <= 0 || !Update.begin(contentLength)) {
    return false;
  }

  mbedtls_sha256_context shaContext;
  mbedtls_sha256_init(&shaContext);
  mbedtls_sha256_starts(&shaContext, 0);

  uint8_t buffer[1024];
  int bytesRemaining = contentLength;
  uint32_t lastProgressMs = millis();

  while (bytesRemaining > 0) {
    size_t available = stream->available();

    if (available == 0) {
      if (millis() - lastProgressMs > HTTP_TIMEOUT_MS) {
        Update.abort();
        mbedtls_sha256_free(&shaContext);
        return false;
      }
      delay(1);
      continue;
    }

    size_t readSize = min(available, sizeof(buffer));
    readSize = min(readSize, static_cast<size_t>(bytesRemaining));
    int bytesRead = stream->readBytes(buffer, readSize);

    if (bytesRead <= 0) {
      Update.abort();
      mbedtls_sha256_free(&shaContext);
      return false;
    }

    if (Update.write(buffer, bytesRead) != static_cast<size_t>(bytesRead)) {
      Update.abort();
      mbedtls_sha256_free(&shaContext);
      return false;
    }

    mbedtls_sha256_update(&shaContext, buffer, bytesRead);
    bytesRemaining -= bytesRead;
    lastProgressMs = millis();
  }

  uint8_t digest[32];
  mbedtls_sha256_finish(&shaContext, digest);
  mbedtls_sha256_free(&shaContext);

  if (bytesToHex(digest, sizeof(digest)) != normalizedSha256(expectedSha256)) {
    Update.abort();
    return false;
  }

  return Update.end() && Update.isFinished();
}

bool getFirmwareMetadata(FirmwareMetadata &metadata) {
  HTTPClient http;
  http.setTimeout(HTTP_TIMEOUT_MS);
  http.begin(FIRMWARE_METADATA_URL);

  int httpCode = http.GET();
  if (httpCode != HTTP_CODE_OK) {
    http.end();
    return false;
  }

  String payload = http.getString();
  http.end();

  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, payload);
  if (error) {
    return false;
  }

  metadata.version = String(doc["version"] | "");
  metadata.url = String(doc["url"] | "");
  metadata.sha256 = String(doc["sha256"] | "");
  if (metadata.sha256.length() == 0) {
    metadata.sha256 = String(doc["hash"] | "");
  }
  metadata.force = doc["force"] | false;
  if (!metadata.force) {
    metadata.force = doc["force_update"] | false;
  }

  return metadata.version.length() > 0 && metadata.url.length() > 0;
}

bool shouldApplyFirmwareUpdate(const FirmwareMetadata &metadata) {
  if (!isSha256Hex(metadata.sha256)) {
    return false;
  }

  if (metadata.force) {
    return metadata.version != CURRENT_FIRMWARE_VERSION;
  }

  return compareVersions(metadata.version, CURRENT_FIRMWARE_VERSION) > 0;
}

bool performOTA(const FirmwareMetadata &metadata) {
  WiFiClientSecure client;
  client.setInsecure();

  HTTPClient http;
  http.setTimeout(HTTP_TIMEOUT_MS);
  if (!http.begin(client, metadata.url)) {
    return false;
  }

  int httpCode = http.GET();
  if (httpCode != HTTP_CODE_OK) {
    http.end();
    return false;
  }

  bool updated = writeFirmwareStreamWithHash(http.getStreamPtr(), http.getSize(), metadata.sha256);
  http.end();

  if (updated) {
    delay(1000);
    ESP.restart();
  }

  return updated;
}

void setupFirmware() {
  bootCount++;
  wakeupCause = esp_sleep_get_wakeup_cause();
  resetReason = esp_reset_reason();

  analogReadResolution(12);
  pinMode(3, INPUT);
  pinMode(1, INPUT);
  pinMode(0, INPUT);

  Wire.begin();
  Wire.setClock(100000);
  initSensors();

  esp_deep_sleep_enable_gpio_wakeup(1 << INTERRUPT_PIN, ESP_GPIO_WAKEUP_GPIO_HIGH);

  WiFi.mode(WIFI_STA);
  wifiTelemetry = setupWiFi();

  if (!wifiTelemetry.connected) {
    enterDeepSleep();
  }

  FirmwareMetadata metadata;
  if (getFirmwareMetadata(metadata) && shouldApplyFirmwareUpdate(metadata)) {
    performOTA(metadata);
  }
}

bool firstRun = true;

void loopFirmware() {
  float voltagePin3 = analogRead(3) * (3.3 / 4095.0);
  float voltagePin1 = analogRead(1) * (3.3 / 4095.0);
  float voltagePin0 = analogRead(0) * (3.3 / 4095.0);

  double voltage = lipo.getVoltage();
  double soc = lipo.getSOC();
  bool batteryAlert = lipo.getAlert();
  (void)voltage;
  (void)soc;
  (void)batteryAlert;

  if (!firstRun) {
    if (!myImager.setPowerMode(SF_VL53L5CX_POWER_MODE::WAKEUP)) {
      enterDeepSleep();
    }
    delay(100);
  } else {
    firstRun = false;
  }

  delay(100);
  processRangingData();
  myImager.stopRanging();
  myImager.setPowerMode(SF_VL53L5CX_POWER_MODE::SLEEP);

  float humidity = climateSensor.readHumidity();
  float temperature = climateSensor.readTemperature();

  accelerometer.getSensorData();
  uint16_t interruptStatus = 0;
  motionTelemetry.interruptStatusReady = accelerometer.getInterruptStatus(&interruptStatus) == BMA400_OK;
  motionTelemetry.interruptStatus = interruptStatus;
  motionTelemetry.wakeupInterrupt = motionTelemetry.interruptStatusReady && (interruptStatus & BMA400_ASSERTED_WAKEUP_INT);
  motionTelemetry.activityInterrupt = motionTelemetry.interruptStatusReady && (interruptStatus & BMA400_ASSERTED_GEN2_INT);

  sendDataToServer(temperature, humidity, accelerometer.data.accelX, accelerometer.data.accelY, accelerometer.data.accelZ,
                   voltagePin0, voltagePin3, voltagePin1);

  enterDeepSleep();
}

WifiTelemetry setupWiFi() {
  WifiTelemetry telemetry;
  uint32_t startedMs = millis();

  WiFiManager wifiManager;
  wifiManager.setConnectTimeout(WIFI_CONNECT_TIMEOUT_S);
  wifiManager.setConfigPortalTimeout(WIFI_CONFIG_PORTAL_TIMEOUT_S);

  telemetry.connected = wifiManager.autoConnect("relod", "password") && WiFi.status() == WL_CONNECTED;
  telemetry.connectMs = millis() - startedMs;
  if (telemetry.connected) {
    telemetry.rssiDbm = WiFi.RSSI();
  }

  return telemetry;
}

void initSensors() {
  Wire.begin();
  Wire.setClock(400000);

  myImager.begin(0x29, Wire);
  myImager.setPowerMode(SF_VL53L5CX_POWER_MODE::WAKEUP);
  delay(10);
  myImager.setResolution(8 * 8);
  myImager.startRanging();

  climateSensor.begin();

  while (accelerometer.beginI2C(BMA400_I2C_ADDRESS) != BMA400_OK) {
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
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), bma400InterruptHandler, RISING);

  if (lipo.begin()) {
    lipo.quickStart();
    lipo.setThreshold(20);
  }
}

void processRangingData() {
  int imageResolution = myImager.getResolution();
  int imageWidth = sqrt(imageResolution);
  int dataIndex = 0;
  uint16_t validDistances[DISTANCE_GRID_SIZE];
  uint32_t signalTotal = 0;
  uint32_t ambientTotal = 0;
  uint16_t reflectanceTotal = 0;

  memset(distanceGrid, 0, sizeof(distanceGrid));
  distanceQuality = DistanceQuality();
  distanceQuality.dataReady = myImager.isDataReady();

  if (!distanceQuality.dataReady || !myImager.getRangingData(&measurementData)) {
    return;
  }

  distanceQuality.readOk = true;

  for (int y = 0; y <= imageWidth * (imageWidth - 1); y += imageWidth) {
    for (int x = imageWidth - 1; x >= 0; x--) {
      if (dataIndex >= DISTANCE_GRID_SIZE) {
        continue;
      }

      int sensorIndex = x + y;
      int distanceMm = measurementData.distance_mm[sensorIndex];
      uint8_t targetStatus = measurementData.target_status[sensorIndex];
      uint8_t targetCount = measurementData.nb_target_detected[sensorIndex];

      distanceGrid[dataIndex++] = distanceMm;

      if (distanceMm <= 0) {
        distanceQuality.zeroCount++;
      }

      if (distanceMm >= DISTANCE_FAR_MM) {
        distanceQuality.farCount++;
      }

      if (targetCount > 0) {
        distanceQuality.targetDetectedCount++;
      }

      if (!isUsableTargetStatus(targetStatus)) {
        distanceQuality.badStatusCount++;
      }

      if (distanceMm > 0 && distanceMm < DISTANCE_FAR_MM && isUsableTargetStatus(targetStatus)) {
        validDistances[distanceQuality.validCount++] = static_cast<uint16_t>(distanceMm);
      }

      signalTotal += measurementData.signal_per_spad[sensorIndex];
      ambientTotal += measurementData.ambient_per_spad[sensorIndex];
      reflectanceTotal += measurementData.reflectance[sensorIndex];
    }
  }

  distanceQuality.spreadMm = percentileSpread(validDistances, distanceQuality.validCount);
  distanceQuality.signalAvg = signalTotal / DISTANCE_GRID_SIZE;
  distanceQuality.ambientAvg = ambientTotal / DISTANCE_GRID_SIZE;
  distanceQuality.reflectanceAvg = reflectanceTotal / DISTANCE_GRID_SIZE;
}

void appendTelemetry(JsonDocument &jsonDoc) {
  jsonDoc["schema_version"] = SCHEMA_VERSION;
  jsonDoc["boot_count"] = bootCount;
  jsonDoc["measurement_sequence"] = measurementSequence;
  jsonDoc["sleep_interval_s"] = SLEEP_INTERVAL_S;
  jsonDoc["wake_cause"] = wakeupCauseToString(wakeupCause);
  jsonDoc["reset_reason"] = resetReasonToString(resetReason);
  jsonDoc["motion_wake"] = isMotionWake(wakeupCause);
  jsonDoc["battery_alert"] = lipo.getAlert() != 0;
  jsonDoc["wifi_rssi_dbm"] = wifiTelemetry.rssiDbm;
  jsonDoc["wifi_connect_ms"] = wifiTelemetry.connectMs;

  // These describe the previous completed POST. Current POST status is only known after this payload leaves.
  jsonDoc["http_status"] = previousHttpStatus;
  jsonDoc["post_ms"] = previousPostMs;
  jsonDoc["retry_count"] = previousRetryCount;
  jsonDoc["free_heap"] = ESP.getFreeHeap();

  jsonDoc["distance_data_ready"] = distanceQuality.dataReady;
  jsonDoc["distance_read_ok"] = distanceQuality.readOk;
  jsonDoc["distance_valid_count"] = distanceQuality.validCount;
  jsonDoc["distance_zero_count"] = distanceQuality.zeroCount;
  jsonDoc["distance_far_count"] = distanceQuality.farCount;
  jsonDoc["distance_bad_status_count"] = distanceQuality.badStatusCount;
  jsonDoc["distance_spread_mm"] = distanceQuality.spreadMm;
  jsonDoc["distance_target_detected_count"] = distanceQuality.targetDetectedCount;
  jsonDoc["distance_signal_avg"] = distanceQuality.signalAvg;
  jsonDoc["distance_ambient_avg"] = distanceQuality.ambientAvg;
  jsonDoc["distance_reflectance_avg"] = distanceQuality.reflectanceAvg;

  jsonDoc["bma400_interrupt_status"] = motionTelemetry.interruptStatus;
  jsonDoc["bma400_interrupt_status_ready"] = motionTelemetry.interruptStatusReady;
  jsonDoc["bma400_wakeup_interrupt"] = motionTelemetry.wakeupInterrupt;
  jsonDoc["bma400_activity_interrupt"] = motionTelemetry.activityInterrupt;
}

bool sendDataToServer(float temperature, float humidity, float accelerationX, float accelerationY, float accelerationZ,
                      float voltageRed, float voltageYellow, float voltageGreen) {
  measurementSequence++;

  HTTPClient http;
  http.setTimeout(HTTP_TIMEOUT_MS);
  http.begin(MEASUREMENT_URL);
  http.addHeader("Content-Type", "application/json");

  double voltage = lipo.getVoltage();
  double soc = lipo.getSOC();
  int finalStatus = 0;
  uint32_t finalPostMs = 0;
  uint8_t retryCount = 0;
  bool sent = false;

  for (uint8_t attempt = 0; attempt < MAX_POST_ATTEMPTS; attempt++) {
    JsonDocument jsonDoc;
    jsonDoc["device_id"] = readMacAddress();
    jsonDoc["temperature"] = temperature;
    jsonDoc["humidity"] = humidity;
    jsonDoc["voltage"] = voltage;
    jsonDoc["soc"] = soc;
    jsonDoc["voltage_red"] = voltageRed;
    jsonDoc["voltage_yellow"] = voltageYellow;
    jsonDoc["voltage_green"] = voltageGreen;
    jsonDoc["firmware_version"] = CURRENT_FIRMWARE_VERSION;

    JsonArray distanceArray = jsonDoc["distance_mm"].to<JsonArray>();
    for (uint8_t i = 0; i < DISTANCE_GRID_SIZE; i++) {
      distanceArray.add(distanceGrid[i]);
    }

    jsonDoc["acceleration_x"] = accelerationX;
    jsonDoc["acceleration_y"] = accelerationY;
    jsonDoc["acceleration_z"] = accelerationZ;
    appendTelemetry(jsonDoc);

    String requestBody;
    serializeJson(jsonDoc, requestBody);

    uint32_t startedMs = millis();
    finalStatus = http.POST(requestBody);
    finalPostMs = millis() - startedMs;

    if (finalStatus >= 200 && finalStatus < 300) {
      http.getString();
      sent = true;
      break;
    }

    if (finalStatus > 0 && finalStatus < 500) {
      http.getString();
      break;
    }

    retryCount = attempt + 1;
    delay(POST_RETRY_DELAY_MS);
  }

  previousHttpStatus = finalStatus;
  previousPostMs = finalPostMs;
  previousRetryCount = retryCount;

  http.end();
  return sent;
}

String readMacAddress() {
  uint8_t baseMac[6];
  esp_wifi_get_mac(WIFI_IF_STA, baseMac);
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           baseMac[0], baseMac[1], baseMac[2], baseMac[3], baseMac[4], baseMac[5]);
  return String(macStr);
}

void enterDeepSleep() {
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  esp_sleep_enable_timer_wakeup(SLEEP_INTERVAL_S * 1000000ULL);
  esp_deep_sleep_start();
}

void bma400InterruptHandler() {
  interruptOccurred = true;
}

}  // namespace

void setup() {
  setupFirmware();
}

void loop() {
  loopFirmware();
}
