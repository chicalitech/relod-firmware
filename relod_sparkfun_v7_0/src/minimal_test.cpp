#include <Arduino.h>
#include <WiFi.h>

void setup() {
  Serial.begin(115200);
  delay(2500);

  pinMode(9, INPUT);
  Serial.println();
  Serial.println("=== ESP32-C6 MINIMAL DIAGNOSTIC ===");
  Serial.printf("Reset reason: %d\n", static_cast<int>(esp_reset_reason()));
  Serial.printf("GPIO9 level after boot: %d\n", digitalRead(9));

  WiFi.mode(WIFI_STA);
  WiFi.disconnect(false, false);
  Serial.println("Scanning WiFi...");
  const int networkCount = WiFi.scanNetworks();
  Serial.printf("WiFi networks found: %d\n", networkCount);
  WiFi.scanDelete();
}

void loop() {
  static unsigned long heartbeat = 0;
  Serial.printf("Heartbeat %lu, GPIO9=%d, free heap=%u\n", heartbeat++,
                digitalRead(9), ESP.getFreeHeap());
  delay(1000);
}
