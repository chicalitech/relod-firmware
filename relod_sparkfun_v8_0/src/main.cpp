// Relod v8: qualify lid orientation before ranging, report briefly, then sleep.
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiManager.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <SPI.h>
#include <SparkFun_VL53L5CX_Library.h>
#include <SparkFun_BMA400_Arduino_Library.h>
#include <SparkFun_MAX1704x_Fuel_Gauge_Arduino_Library.h>
#include <Adafruit_Si7021.h>
#include <Adafruit_ThinkInk.h>
#include <Preferences.h>
#include <Update.h>
#include <esp_mac.h>
#include <esp_rtc_time.h>
#include <esp_sleep.h>
#include <esp_sntp.h>
#include <mbedtls/sha256.h>
#include <time.h>
#include <type_traits>
#include "power_policy.h"

constexpr char kVersion[] = "8.0.0";
constexpr char kMeasurementUrl[] = "https://relod.fly.dev/measurement";
constexpr char kMetadataUrl[] = "https://relod.fly.dev/latest_firmware";
constexpr char kTimeZone[] = "PST8PDT,M3.2.0,M11.1.0";
constexpr uint64_t kDayMs = 24ULL * 60 * 60 * 1000;
constexpr uint32_t kNetworkBudgetMs = 30000;
constexpr uint32_t kGateWaitMs = 2000;
constexpr uint32_t kRtcMagic = 0x524C0801;
constexpr uint8_t kMinValidZones = 8;
constexpr int kMotionPin = 3;
#ifndef EPD_BUSY_PIN
#define EPD_BUSY_PIN 18
#endif
#ifndef LID_CLOSED_X
#define LID_CLOSED_X 0.0f
#endif
#ifndef LID_CLOSED_Y
#define LID_CLOSED_Y 0.0f
#endif
#ifndef LID_CLOSED_Z
#define LID_CLOSED_Z 1.0f
#endif
#ifndef RELOD_DEBUG
#define RELOD_DEBUG 0
#endif
// Calibrate this direction with the accelerometer physically mounted on the
// closed lid. +Z is a starting assumption, not automatic calibration.
constexpr relod::LidConfig kLidConfig{
    {LID_CLOSED_X, LID_CLOSED_Y, LID_CLOSED_Z}, 12.0f, 0.035f, 750};

// The installed driver has an unbounded BUSY loop. Bound it without patching
// a downloaded library; later waits in this wake return immediately on fault.
class BatteryDisplay : public ThinkInk_213_Grayscale4_MFGN {
 public:
  BatteryDisplay() : ThinkInk_213_Grayscale4_MFGN(17, 16, 5, -1, EPD_BUSY_PIN, &SPI, 0) {}
  bool timedOut = false;
 protected:
  void busy_wait() override {
    if (timedOut) return;
    const uint32_t started = millis();
    while (digitalRead(EPD_BUSY_PIN)) {
      if (millis() - started >= 12000) { timedOut = true; break; }
      delay(10);
    }
  }
};

struct Sample {
  uint64_t capturedMs;
  uint32_t sequence;
  float temperature, humidity, voltage, soc;
  float red, yellow, green;
  relod::Vector3 acceleration;
  int16_t distance[64];
  uint8_t validZones;
};
struct Retained {
  uint32_t magic, session, sequence;
  uint64_t reportDueMs, otaDueMs, timeSyncDueMs, nextMotionMs;
  uint64_t lastOpenedMs;
  uint32_t skipped, displayHash;
  uint64_t displayRetryMs;
  int lastHttpStatus;
  uint32_t lastPostMs, lastAwakeMs;
  uint8_t lidRetries;
  bool pendingLid, haveLastGood, haveOpened;
  Sample lastGood;
  relod::Queue<Sample, 4> pending;
};
static_assert(sizeof(Retained) < 3000, "Keep the RTC queue small");
static_assert(std::is_trivial<Retained>::value, "RTC state must not reinitialize on wake");
RTC_DATA_ATTR Retained retained{};

SparkFun_VL53L5CX imager;
VL53L5CX_ResultsData ranging;
BMA400 accelerometer;
SFE_MAX1704X battery;
Adafruit_Si7021 climate;
BatteryDisplay display;
bool accelerometerPresent = false;
bool accelerometerReady = false, imagerAttempted = false, imagerRunning = false;
bool batteryReady = false, displayReady = false;
bool coldBoot = true;
bool connectedForDisplay = false;
int rssiForDisplay = -127;
String ssidForDisplay = "Not connected";
String deviceId;
relod::Vector3 acceleration{NAN, NAN, NAN};
float batterySoc = NAN, batteryVoltage = NAN;
uint32_t wifiConnectMs = 0;
volatile bool timeSynced = false;

uint64_t monotonicMs() { return esp_rtc_get_time_us() / 1000; }
bool validTime() { return time(nullptr) >= 1704067200; }

bool readAcceleration() {
  if (!accelerometerReady || accelerometer.getSensorData() != BMA400_OK) return false;
  acceleration = {accelerometer.data.accelX, accelerometer.data.accelY, accelerometer.data.accelZ};
  return true;
}

bool waitForStableLid(relod::LidGate& gate) {
  gate.reset();
  const uint32_t started = millis();
  do {
    const bool ok = readAcceleration();
    if (gate.observe(acceleration, millis(), ok)) return true;
    delay(50); // BMA400 runs at 100 Hz while qualifying the lid.
  } while (millis() - started < kGateWaitMs);
  return false;
}

void initAccelerometer() {
  for (int attempt = 0; attempt < 2 && !accelerometerReady; ++attempt) {
    accelerometerReady = accelerometer.beginI2C(BMA400_I2C_ADDRESS_DEFAULT) == BMA400_OK;
    if (!accelerometerReady) delay(50);
  }
  accelerometerPresent = accelerometerReady;
  if (accelerometerReady) {
    accelerometerReady = accelerometer.setRange(BMA400_RANGE_2G) == BMA400_OK &&
                         accelerometer.setODR(BMA400_ODR_100HZ) == BMA400_OK &&
                         accelerometer.setMode(BMA400_MODE_NORMAL) == BMA400_OK;
    delay(30);
  }
}

// Every range attempt, including partial initialization failures, is cleaned up
// before any network activity. ESP32 deep sleep does not turn off sensor rails.
void sleepImager() {
  if (!imagerAttempted) return;
  if (imagerRunning) imager.stopRanging();
  if (!imager.setPowerMode(SF_VL53L5CX_POWER_MODE::SLEEP)) {
    Serial.println("Warning: distance sensor did not acknowledge sleep; inspect sleep current.");
  }
  imagerRunning = false;
  imagerAttempted = false;
}

bool takeDistanceSample(Sample& sample) {
  imagerAttempted = true;
  bool ok = imager.begin(0x29, Wire);
  ok = ok && imager.setPowerMode(SF_VL53L5CX_POWER_MODE::WAKEUP);
  ok = ok && imager.setResolution(64) && imager.setRangingFrequency(15);
  // Initialization uploads sensor firmware and takes time. Qualify the lid
  // again afterward, then continue checking while awaiting and reading a frame.
  relod::LidGate gate(kLidConfig);
  ok = ok && waitForStableLid(gate);
  imagerRunning = ok && imager.startRanging();
  ok = false;
  const uint32_t started = millis();
  while (imagerRunning && millis() - started < 1000) {
    const bool readOk = readAcceleration();
    if (!gate.observe(acceleration, millis(), readOk)) break;
    if (imager.isDataReady() && imager.getRangingData(&ranging)) {
      const bool finalReadOk = readAcceleration();
      if (!gate.observe(acceleration, millis(), finalReadOk)) break;
      sample.validZones = 0;
      for (int y = 0, output = 0; y < 64; y += 8) {
        for (int x = 7; x >= 0; --x, ++output) {
          const int i = x + y;
          const bool usable = (ranging.target_status[i] == 5 || ranging.target_status[i] == 9) &&
                              ranging.nb_target_detected[i] > 0 &&
                              ranging.distance_mm[i] > 0 && ranging.distance_mm[i] < 3500;
          sample.distance[output] = usable ? ranging.distance_mm[i] : 0;
          sample.validZones += usable;
        }
      }
      ok = sample.validZones >= kMinValidZones;
      sample.acceleration = acceleration;
      break;
    }
    delay(50);
  }
  sleepImager();
  return ok;
}

void readBattery() {
  batteryReady = battery.begin();
  if (!batteryReady) return;
  // Preserve the fuel gauge's ongoing estimate across ESP32 sleep cycles.
  batteryVoltage = battery.getVoltage();
  batterySoc = battery.getSOC();
  if (coldBoot) battery.setThreshold(20);
}

void finishSample(Sample& sample) {
  sample.capturedMs = monotonicMs();
  sample.sequence = ++retained.sequence;
  sample.temperature = sample.humidity = NAN;
  if (climate.begin()) {
    sample.temperature = climate.readTemperature();
    sample.humidity = climate.readHumidity();
  }
  readBattery();
  sample.voltage = batteryVoltage;
  sample.soc = batterySoc;
  sample.red = analogRead(0) * (3.3f / 4095.0f);
  sample.yellow = analogRead(2) * (3.3f / 4095.0f);
  sample.green = analogRead(1) * (3.3f / 4095.0f);
}

String lastOpenedText() {
  if (retained.haveOpened && validTime()) {
    const time_t opened = time(nullptr) - (monotonicMs() - retained.lastOpenedMs) / 1000;
    struct tm local;
    localtime_r(&opened, &local);
    char text[20];
    strftime(text, sizeof(text), "%Y-%m-%d %H:%M", &local);
    return String(text);
  }
  Preferences preferences;
  if (!preferences.begin("relod", true)) return "Not recorded";
  String result = preferences.getString("last_opened", "Not recorded");
  preferences.end();
  return retained.haveOpened ? "Time unavailable" : result;
}

void persistOpenedTime() {
  if (!retained.haveOpened || !validTime()) return;
  const String opened = lastOpenedText();
  Preferences preferences;
  if (preferences.begin("relod", false)) {
    if (preferences.getString("last_opened", "") != opened) preferences.putString("last_opened", opened);
    preferences.end();
  }
}

uint32_t hashText(const String& text) {
  uint32_t result = 2166136261UL;
  for (size_t i = 0; i < text.length(); ++i) result = (result ^ uint8_t(text[i])) * 16777619UL;
  return result;
}

void renderDisplay(const String& stateText, bool portal = false) {
  if (monotonicMs() < retained.displayRetryMs) return;
  const Sample& last = retained.lastGood;
  const String temp = retained.haveLastGood && std::isfinite(last.temperature) ? String(last.temperature, 1) : "--";
  const String humidity = retained.haveLastGood && std::isfinite(last.humidity) ? String(last.humidity, 0) : "--";
  const float soc = std::isfinite(batterySoc) ? batterySoc : (retained.haveLastGood ? last.soc : NAN);
  const String batteryText = std::isfinite(soc) ? String(constrain(int(soc), 0, 100)) + "%" : "--";
  const String opened = lastOpenedText();
  const String network = portal ? stateText : (connectedForDisplay ? ssidForDisplay : "Offline");
  const String age = retained.haveLastGood ? String(static_cast<unsigned long>((monotonicMs() - last.capturedMs) / 60000)) + " min ago" : "None yet";
  const uint32_t hash = hashText(stateText + network + temp + humidity + batteryText + opened + age);
  if (!coldBoot && retained.displayHash == hash) return;
  if (!displayReady) {
    SPI.begin(19, -1, 4, 5);
    display.begin(THINKINK_MONO);
    display.setRotation(2);
    displayReady = true;
  }
  display.clearBuffer();
  display.setTextWrap(false);
  display.setTextColor(EPD_BLACK);
  display.setTextSize(2);
  display.setCursor(6, 4);
  String suffix = deviceId.substring(deviceId.length() - 5);
  suffix.replace(":", "");
  suffix.toUpperCase();
  display.print("RELOD-" + suffix);
  display.setTextSize(1);
  display.setCursor(197, 9);
  display.print(batteryText);
  display.drawFastHLine(6, 24, display.width() - 12, EPD_BLACK);
  display.setCursor(6, 30); display.print(stateText.substring(0, 39));
  display.setCursor(6, 43); display.print("Last range: " + age);
  display.setCursor(6, 56); display.print("WiFi: " + network.substring(0, 30));
  display.setCursor(6, 69); display.print("Temp: " + temp + " C  RH: " + humidity + "%");
  display.setCursor(6, 82); display.print("Opened: " + opened);
  display.setCursor(6, 104); display.print("Tantalizing Turkish");
  // The installed EPD API defaults to display(false), which leaves it awake.
  display.display(true);
  if (display.timedOut) {
    retained.displayRetryMs = monotonicMs() + kDayMs;
    Serial.println("E-ink BUSY timeout; display disabled for 24 h or until reset.");
  } else {
    retained.displayHash = hash;
  }
}

bool connectWiFi() {
  const uint32_t started = millis();
  WiFi.mode(WIFI_STA);
  WiFiManager wm;
  wm.setConnectTimeout(8);
  wm.setConfigPortalTimeout(180);
  wm.setAPClientCheck(false);
  wm.setWebPortalClientCheck(false);
  // A deliberate reset/power cycle is the recovery path. Ordinary deep-sleep
  // wakes never open a hotspot, even if no credentials have been saved yet.
  wm.setEnableConfigPortal(coldBoot);
  wm.setAPCallback([](WiFiManager* manager) {
    const String name = manager->getConfigPortalSSID();
    Serial.println("Setup AP: " + name + " / password: password / 192.168.4.1");
    renderDisplay("Setup: " + name, true);
  });
  String suffix = deviceId;
  suffix.replace(":", "");
  const String apName = "relod-" + suffix.substring(8);
  const bool connected = wm.autoConnect(apName.c_str(), "password");
  wifiConnectMs = millis() - started;
  connectedForDisplay = connected && WiFi.status() == WL_CONNECTED;
  if (connectedForDisplay) {
    ssidForDisplay = WiFi.SSID();
    rssiForDisplay = WiFi.RSSI();
    WiFi.setSleep(true); // Test energy per report on the deployment router.
  }
  return connectedForDisplay;
}

void stopWiFi() {
  if (WiFi.getMode() != WIFI_OFF) {
    WiFi.disconnect(true, false); // Retain credentials.
    WiFi.mode(WIFI_OFF);
  }
}

void syncClockIfDue() {
  if (monotonicMs() < retained.timeSyncDueMs) return;
  retained.timeSyncDueMs = monotonicMs() + 6ULL * 60 * 60 * 1000;
  timeSynced = false;
  sntp_set_time_sync_notification_cb([](struct timeval*) { timeSynced = true; });
  configTzTime(kTimeZone, "pool.ntp.org", "time.nist.gov");
  const uint32_t started = millis();
  while (!timeSynced && millis() - started < 1200) delay(20);
  esp_sntp_stop();
  if (timeSynced) retained.timeSyncDueMs = monotonicMs() + kDayMs;
}

void configureHttp(HTTPClient& http, WiFiClientSecure& client) {
  // Preserves v7's transport policy. Validating server certificates is a
  // separate deployment prerequisite; SHA-256 is not server authentication.
  client.setInsecure();
  client.setHandshakeTimeout(5); // seconds, unlike HTTPClient's millisecond API
  client.setTimeout(3000);
  http.setConnectTimeout(4000);
  http.setTimeout(3000);
  http.setReuse(false);
}

String sampleBody(const Sample& sample, String& id) {
  char suffix[32];
  snprintf(suffix, sizeof(suffix), "-%08lx-%lu", static_cast<unsigned long>(retained.session),
           static_cast<unsigned long>(sample.sequence));
  id = deviceId + suffix;
  JsonDocument doc;
  doc["device_id"] = deviceId;
  doc["firmware_version"] = kVersion;
  doc["temperature"] = sample.temperature;
  doc["humidity"] = sample.humidity;
  doc["voltage"] = sample.voltage;
  doc["soc"] = sample.soc;
  doc["voltage_red"] = sample.red;
  doc["voltage_yellow"] = sample.yellow;
  doc["voltage_green"] = sample.green;
  doc["acceleration_x"] = sample.acceleration.x;
  doc["acceleration_y"] = sample.acceleration.y;
  doc["acceleration_z"] = sample.acceleration.z;
  JsonArray grid = doc["distance_mm"].to<JsonArray>();
  for (const auto value : sample.distance) grid.add(value);
  const uint64_t age = (monotonicMs() - sample.capturedMs) / 1000;
  doc["schema_version"] = 2;
  doc["measurement_id"] = id;
  doc["measurement_sequence"] = sample.sequence;
  doc["measurement_age_s"] = age;
  if (validTime()) doc["measured_at_unix_s"] = static_cast<int64_t>(time(nullptr)) - age;
  else doc["measured_at_unix_s"] = nullptr;
  doc["distance_fresh"] = sample.sequence == retained.sequence && age < 60;
  doc["distance_valid_count"] = sample.validZones;
  doc["lid_horizontal"] = true;
  doc["lid_stable"] = true;
  doc["skipped_measurements"] = retained.skipped;
  doc["queue_dropped"] = retained.pending.dropped;
  doc["queue_depth"] = retained.pending.count;
  doc["wifi_connect_ms"] = wifiConnectMs;
  doc["wifi_rssi_dbm"] = rssiForDisplay;
  doc["previous_http_status"] = retained.lastHttpStatus;
  doc["previous_post_ms"] = retained.lastPostMs;
  doc["previous_awake_ms"] = retained.lastAwakeMs;
  String body;
  serializeJson(doc, body);
  return body;
}

bool sendPending(uint32_t networkStarted) {
  // Total two POST attempts per wake, including backlog. No endless flush on
  // reconnect. Original sample IDs and timestamps survive all retries.
  uint8_t attempts = 0;
  bool sentAny = false;
  while (retained.pending.count && attempts < 2 && millis() - networkStarted < kNetworkBudgetMs - 12000) {
    String id;
    const String body = sampleBody(retained.pending.items[0], id);
    WiFiClientSecure client;
    HTTPClient http;
    configureHttp(http, client);
    if (!http.begin(client, kMeasurementUrl)) break;
    http.addHeader("Content-Type", "application/json");
    http.addHeader("Idempotency-Key", id); // Server support required for dedup.
    const uint32_t started = millis();
    const int status = http.POST(body);
    retained.lastPostMs = millis() - started;
    retained.lastHttpStatus = status;
    ++attempts;
    http.end();
    Serial.printf("POST %s: %d (%lu ms)\n", id.c_str(), status,
                  static_cast<unsigned long>(retained.lastPostMs));
    if (relod::successfulHttp(status)) {
      retained.pending.pop();
      sentAny = true;
    } else {
      if (!relod::retryableHttp(status) || attempts == 2 || status == 429) break;
      if (millis() - networkStarted >= kNetworkBudgetMs - 12250) break;
      delay(250);
    }
  }
  return sentAny;
}

bool readSmallBody(HTTPClient& http, String& body) {
  const int length = http.getSize();
  // HTTP/1.0 metadata request supplies a length or close-delimited body.
  if (length > 4096) return false;
  auto* stream = http.getStreamPtr();
  const uint32_t started = millis();
  while (millis() - started < 4000) {
    while (stream->available()) {
      if (body.length() >= 4096) return false;
      const int c = stream->read();
      if (c < 0) break;
      body += static_cast<char>(c);
      if (length >= 0 && body.length() == static_cast<size_t>(length)) return true;
      if (millis() - started >= 4000) return false;
    }
    if (!stream->connected()) return length < 0 || body.length() == static_cast<size_t>(length);
    delay(1);
  }
  return false;
}

bool installVerifiedFirmware(const String& url, const String& expectedHash) {
  WiFiClientSecure client;
  HTTPClient http;
  configureHttp(http, client);
  if (!http.begin(client, url) || http.GET() != HTTP_CODE_OK) { http.end(); return false; }
  const int length = http.getSize();
  if (length <= 0 || static_cast<uint32_t>(length) > ESP.getFreeSketchSpace() || !Update.begin(length)) {
    http.end();
    return false;
  }
  mbedtls_sha256_context context;
  mbedtls_sha256_init(&context);
  bool ok = mbedtls_sha256_starts(&context, 0) == 0;
  auto* stream = http.getStreamPtr();
  int remaining = length;
  const uint32_t started = millis();
  uint32_t lastProgress = started;
  uint8_t buffer[1024];
  while (ok && remaining > 0) {
    if (millis() - started >= 120000 || millis() - lastProgress >= 5000) { ok = false; break; }
    const size_t available = stream->available();
    if (!available) { delay(1); continue; }
    const size_t amount = min(min(available, sizeof(buffer)), static_cast<size_t>(remaining));
    const int read = stream->read(buffer, amount);
    if (read <= 0 || Update.write(buffer, read) != static_cast<size_t>(read)) { ok = false; break; }
    ok = mbedtls_sha256_update(&context, buffer, read) == 0;
    remaining -= read;
    lastProgress = millis();
  }
  uint8_t digest[32];
  if (ok) ok = mbedtls_sha256_finish(&context, digest) == 0;
  mbedtls_sha256_free(&context);
  char hex[65]{};
  if (ok) {
    for (size_t i = 0; i < sizeof(digest); ++i) snprintf(hex + 2 * i, 3, "%02x", digest[i]);
    ok = expectedHash.equalsIgnoreCase(hex);
  }
  if (ok) ok = Update.end() && Update.isFinished();
  if (!ok) Update.abort();
  http.end();
  return ok;
}

bool checkFirmwareIfDue() {
  if (monotonicMs() < retained.otaDueMs || retained.pending.count ||
      !batteryReady || !std::isfinite(batterySoc) || !std::isfinite(batteryVoltage) ||
      batterySoc < 30 || batteryVoltage < 3.65f) return false;
  retained.otaDueMs = monotonicMs() + kDayMs; // Failed checks cannot repeat on each motion wake.
  WiFiClientSecure client;
  HTTPClient http;
  configureHttp(http, client);
  http.useHTTP10(true);
  if (!http.begin(client, kMetadataUrl) || http.GET() != HTTP_CODE_OK) { http.end(); return false; }
  String body;
  const bool read = readSmallBody(http, body);
  http.end();
  JsonDocument doc;
  if (!read || deserializeJson(doc, body)) return false;
  const String board = doc["board"] | "";
  if (board != "sparkfun_esp32c6_thing_plus") return false;
  const String version = doc["version"] | "";
  const String url = doc["url"] | "";
  String hash = doc["sha256"] | "";
  if (hash.isEmpty()) hash = String(doc["hash"] | "");
  if (!relod::newerVersion(version.c_str(), kVersion) || !url.startsWith("https://") ||
      !relod::sha256Hex(hash.c_str())) return false;
  Serial.println("Downloading newer firmware with SHA-256 verification.");
  return installVerifiedFirmware(url, hash);
}

bool armMotionWake() {
  if (!accelerometerReady) {
    if (accelerometerPresent) accelerometer.setMode(BMA400_MODE_LOW_POWER);
    return false;
  }
  bma400_wakeup_conf config{};
  config.wakeup_ref_update = BMA400_UPDATE_ONE_TIME;
  config.sample_count = BMA400_SAMPLE_COUNT_2;
  config.wakeup_axes_en = BMA400_AXIS_XYZ_EN;
  config.int_wkup_threshold = 6;
  config.int_chan = BMA400_INT_CHANNEL_1;
  bool ok = accelerometer.setWakeupInterrupt(&config) == BMA400_OK;
  ok = (accelerometer.setInterruptPinMode(BMA400_INT_CHANNEL_1, BMA400_INT_PUSH_PULL_ACTIVE_1) == BMA400_OK) && ok;
  uint16_t status = 0;
  accelerometer.getInterruptStatus(&status);
  ok = (accelerometer.enableInterrupt(BMA400_AUTO_WAKEUP_EN, true) == BMA400_OK) && ok;
  ok = (accelerometer.setMode(BMA400_MODE_LOW_POWER) == BMA400_OK) && ok;
  delay(100); // Allow the 25 Hz low-power reference to settle.
  // Level-sensitive wake must not be armed while the line is already high.
  return ok && digitalRead(kMotionPin) == LOW;
}

void enterSleep(uint64_t sleepMs, bool allowMotion) {
  sleepImager();
  stopWiFi();
  const bool motionReady = armMotionWake();
  if (allowMotion && motionReady) {
    const auto error = esp_deep_sleep_enable_gpio_wakeup(1ULL << kMotionPin, ESP_GPIO_WAKEUP_GPIO_HIGH);
    if (error != ESP_OK) Serial.printf("Motion wake configuration failed: %d\n", error);
  }
  // Follow-up waits deliberately use timer-only wake to coalesce motion storms.
  esp_sleep_enable_timer_wakeup(sleepMs * 1000ULL);
  retained.lastAwakeMs = millis();
  Serial.printf("Sleep %lu s, motion=%s, pending=%u, awake=%lu ms\n",
                static_cast<unsigned long>(sleepMs / 1000), allowMotion && motionReady ? "on" : "off",
                retained.pending.count, static_cast<unsigned long>(retained.lastAwakeMs));
  Serial.flush();
  esp_deep_sleep_start();
}

void setup() {
  Serial.begin(115200);
  if (RELOD_DEBUG) delay(1500);
  coldBoot = esp_reset_reason() != ESP_RST_DEEPSLEEP;
  const uint64_t now = monotonicMs();
  if (coldBoot || retained.magic != kRtcMagic) {
    retained = Retained{};
    retained.magic = kRtcMagic;
    retained.session = esp_random();
    retained.reportDueMs = now;
  }
  setenv("TZ", kTimeZone, 1);
  tzset();
  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_WIFI_STA); // No radio startup needed for identity.
  char macText[18];
  snprintf(macText, sizeof(macText), "%02x:%02x:%02x:%02x:%02x:%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  deviceId = macText;
  Serial.printf("Relod %s, wake=%d\n", kVersion, esp_sleep_get_wakeup_cause());
  analogReadResolution(12);
  Wire.begin();
  Wire.setClock(400000);
  Wire.setTimeOut(50);
  pinMode(kMotionPin, INPUT_PULLDOWN);
  initAccelerometer();
  const bool motionWake = esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_GPIO;
  if (motionWake && now < retained.nextMotionMs) {
    enterSleep(retained.nextMotionMs - now, false);
    return;
  }
  retained.nextMotionMs = now + 30000; // Bound repeated motion even when the lid never qualifies.
  if (now >= retained.reportDueMs) {
    retained.reportDueMs = now + relod::kReportIntervalMs;
    retained.lidRetries = 0;
  }
  relod::LidGate gate(kLidConfig);
  const bool horizontal = accelerometerReady && waitForStableLid(gate);
  Serial.printf("Lid acceleration: %.3f %.3f %.3f g; stable=%s\n", acceleration.x,
                acceleration.y, acceleration.z, horizontal ? "yes" : "no");
  if (!horizontal && motionWake && !retained.pendingLid && readAcceleration() &&
      !relod::horizontal(acceleration, kLidConfig)) {
    retained.lastOpenedMs = now;
    retained.haveOpened = true;
  }
  Sample sample{};
  const bool measured = horizontal && takeDistanceSample(sample);
  if (measured) {
    finishSample(sample);
    retained.lastGood = sample;
    retained.haveLastGood = true;
    retained.pending.push(sample);
    retained.pendingLid = false;
    retained.lidRetries = 0;
    retained.nextMotionMs = monotonicMs() + 30000;
  } else {
    // Cold hardware starts in high-power idle, and firmware may have been
    // replaced while the previous image was still ranging. Initialize once
    // solely to issue a reliable sleep command; never start a range here.
    if (coldBoot && !horizontal) {
      imagerAttempted = true;
      imager.begin(0x29, Wire);
      sleepImager();
    }
    ++retained.skipped;
    retained.pendingLid = true;
  }
  const String stateText = measured ? "Lid ready - range captured" :
      (!accelerometerReady ? "No accelerometer - range skipped" :
       (horizontal ? "Range unavailable - will retry" : "Waiting for level, still lid"));
  bool updated = false;
  // Cold boot still offers setup/recovery even with a tilted or absent lid.
  if (measured || coldBoot) {
    if (!measured) readBattery();
    if (connectWiFi()) {
      const uint32_t networkStarted = millis();
      syncClockIfDue();
      const bool sent = sendPending(networkStarted);
      persistOpenedTime();
      // OTA is separate from the reporting budget and capped at 120 s streaming.
      if (sent && millis() - networkStarted < kNetworkBudgetMs - 12000) updated = checkFirmwareIfDue();
    }
  }
  stopWiFi(); // Display refresh does not need the radio.
  renderDisplay(stateText);
  if (updated) {
    // Sensors and display have already been put to sleep before rebooting.
    ESP.restart();
    return;
  }
  const uint64_t sleepMs = relod::nextSleepMs(monotonicMs(), retained.reportDueMs,
                                             retained.pendingLid, retained.lidRetries);
  const bool shortFollowup = retained.pendingLid && sleepMs <= relod::kLidRetryMs;
  enterSleep(sleepMs, !shortFollowup);
}

void loop() { delay(1000); } // setup ends in deep sleep or a verified OTA reboot.
