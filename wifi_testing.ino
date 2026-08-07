#include <WiFi.h>

const char* SSID = "Access Point Name";  // change to your AP name
const char* PASS = "Password";        // change to your AP password

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== ESP32 bare WiFi test ===");
  Serial.print("Reset reason: ");
  Serial.println(esp_reset_reason());   // 5 = BROWNOUT
  Serial.print("MAC: ");
  Serial.println(WiFi.macAddress());

  // Scan first — proves whether the AP is even visible
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(200);
  Serial.println("Scanning...");
  int n = WiFi.scanNetworks();
  for (int i = 0; i < n; i++) {
    Serial.printf("%2d) %-28s ch:%2d  %4d dBm  enc:%d\n",
      i + 1, WiFi.SSID(i).c_str(), WiFi.channel(i),
      WiFi.RSSI(i), WiFi.encryptionType(i));
  }
  // enc: 0=OPEN 2=WPA_PSK 3=WPA2_PSK 4=WPA_WPA2 6=WPA3_PSK 7=WPA2_WPA3

  Serial.printf("\nConnecting to '%s'...\n", SSID);
  WiFi.begin(SSID, PASS);
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 25000) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("SUCCESS. IP: "); Serial.println(WiFi.localIP());
    Serial.print("RSSI: ");        Serial.println(WiFi.RSSI());
  } else {
    Serial.print("FAILED. status="); Serial.println(WiFi.status());
  }
}

void loop() {}