/*  ESP32 DevKit — 5 Sensor Monitor + WiFi + Adafruit IO
   ─────────────────────────────────────────────────────
   ESP32 has 18 analog pins — NO MUX!
   All MQ sensors use AOUT directly.
   ─────────────────────────────────────────────────────
   DHT22   SIG  → GPIO 4
   MQ-135  AOUT → GPIO 34 (input only)
   MQ-3    AOUT → GPIO 35 (input only)
   MQ-4    AOUT → GPIO 32
   MQ-7    AOUT → GPIO 33
   OLED    SDA  → GPIO 21
   OLED    SCL  → GPIO 22
   ─────────────────────────────────────────────────────
   ADC: 12-bit (0–4095) @ 3.3V max
   IMPORTANT: Add 10kΩ/20kΩ voltage
   divider on every MQ AOUT wire!
   ─────────────────────────────────────────────────────
   NEW in this version:
   • WiFi setup via CAPTIVE PORTAL (no hardcoded SSID).
     ON EVERY BOOT the board first tries the WiFi it saved
     last time. If that network IS available it connects
     automatically and goes straight to monitoring.
     If the saved network is NOT available (out of range,
     router off, or nothing saved yet) it AUTOMATICALLY
     opens a hotspot "ESP32-AirMonitor" (pass: 12345678) —
     no button press needed. Connect with your phone, a
     config page pops up, pick a new network, and it saves
     it for next time.
     OPTIONAL: holding the FLASH/BOOT button (GPIO 0) at
     power-up wipes the saved WiFi and forces the portal
     even if the old network would have worked.
   • Sends all 6 readings to ADAFRUIT IO (free) via MQTT,
     every UPLOAD_MS, non-blocking.
   ─────────────────────────────────────────────────────
   Libraries (install via Library Manager):
   • Adafruit SSD1306 + Adafruit GFX
   • DHT sensor library + Adafruit Unified Sensor
   • WiFiManager  by tzapu
   • Adafruit MQTT Library
   ───────────────────────────────────────────────────── */

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <DHT.h>

// ── Networking ──────────────────────────────────────────
#include <WiFi.h>
#include <WiFiManager.h> // tzapu/WiFiManager
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

/*  ┌────────────────────────────────────────────────────┐
    │  FILL IN YOUR ADAFRUIT IO CREDENTIALS               │
    │  Get them free at https://io.adafruit.com           │
    │  (top-right "key" icon = Username + Active Key)     │
    └────────────────────────────────────────────────────┘ */
#define AIO_USERNAME ""
#define AIO_KEY ""
#define AIO_SERVER "io.adafruit.com"
#define AIO_SERVERPORT 1883 // 1883 = plain, 8883 = TLS

// Config-portal access-point (shown if no WiFi saved)
#define AP_NAME "ESP32-AirMonitor"
#define AP_PASS "12345678" // min 8 chars; "" for open AP
#define PORTAL_BUTTON 0    // FLASH/BOOT button = force portal

// How often to push data to Adafruit IO.
// Free tier = 30 data points / minute TOTAL. We send 6 feeds,
// so 20 s -> 6*3 = 18 pts/min (safe). Don't go below ~15 s.
#define UPLOAD_MS 20000UL

// ── Display / sensors (unchanged) ───────────────────────
#define SCREEN_W 128
#define SCREEN_H 64
#define OLED_ADDR 0x3C
#define SDA_PIN 21
#define SCL_PIN 22
#define DHT_PIN 4
#define DHT_TYPE DHT11
// All MQ sensors → direct AOUT (via 10kΩ/20kΩ divider!)
#define MQ135_PIN 34
#define MQ3_PIN 35
#define MQ4_PIN 32
#define MQ7_PIN 33
// ESP32 ADC: 12-bit = 0-4095
#define ADC_MAX 4095
#define AIR_ALERT 2000
#define ALC_ALERT 1600
#define MQ4_ALERT 1200
#define MQ7_ALERT 800

Adafruit_SSD1306 oled(SCREEN_W, SCREEN_H, &Wire, -1);
DHT dht(DHT_PIN, DHT_TYPE);

// ── MQTT client + feeds ─────────────────────────────────
WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT,
                          AIO_USERNAME, AIO_KEY);

// Feed paths: <username>/feeds/<feedname>
// Create these feeds in Adafruit IO (or they auto-create on
// first publish). Names must match exactly.
Adafruit_MQTT_Publish feedTemp = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/temperature");
Adafruit_MQTT_Publish feedHum = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/humidity");
Adafruit_MQTT_Publish feedAir = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/air-quality");
Adafruit_MQTT_Publish feedAlc = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/alcohol");
Adafruit_MQTT_Publish feedCH4 = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/methane");
Adafruit_MQTT_Publish feedCO = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/carbon-monoxide");

// ── UI state (unchanged) ────────────────────────────────
int page = 0;
unsigned long lastPage = 0;
#define PAGE_MS 3000
#define PAGES 5
bool alertFlip = false;

// ── Cloud state ─────────────────────────────────────────
unsigned long lastUpload = 0;
unsigned long lastReconnect = 0;
bool wifiOK = false;

int rawToPPM(int raw, int maxPPM)
{
  return map(raw, 0, ADC_MAX, 0, maxPPM);
}
const char *airLabel(int ppm)
{
  if (ppm < 200)
    return "GOOD";
  if (ppm < 400)
    return "MODERATE";
  if (ppm < 600)
    return "UNHEALTHY";
  if (ppm < 800)
    return "DANGEROUS";
  return "HAZARDOUS";
}
void drawBar(int x, int y, int w, int h, int pct)
{
  oled.drawRect(x, y, w, h, WHITE);
  oled.fillRect(x + 1, y + 1, (w - 2) * constrain(pct, 0, 100) / 100, h - 2, WHITE);
}
void titleBar(const char *t)
{
  oled.setTextSize(1);
  oled.setTextColor(WHITE);
  oled.setCursor(0, 4);
  oled.print(t);
  // WiFi indicator: filled dot = connected, hollow = offline
  wifiOK ? oled.fillCircle(122, 7, 3, WHITE)
         : oled.drawCircle(122, 7, 3, WHITE);
  for (int i = 0; i < PAGES; i++)
  {
    int dx = 76 + i * 6;
    i == page ? oled.fillCircle(dx, 7, 2, WHITE)
              : oled.drawCircle(dx, 7, 2, WHITE);
  }
}

// Page 0: Temp & Humidity
void pageDHT(float t, float h, float hi)
{
  oled.clearDisplay();
  titleBar("TEMP & HUM");
  oled.setTextSize(1);
  oled.setCursor(0, 18);
  oled.print("T:");
  oled.setTextSize(2);
  oled.setCursor(16, 14);
  oled.print(t, 1);
  oled.print("C");
  drawBar(0, 30, 128, 6, constrain(t / 50 * 100, 0, 100));
  oled.setTextSize(1);
  oled.setCursor(0, 40);
  oled.print("H:");
  oled.print(h, 1);
  oled.print("%");
  drawBar(0, 50, 128, 6, h);
  oled.setCursor(70, 40);
  oled.print("HI:");
  oled.print(hi, 1);
  oled.display();
}
// Page 1: Air Quality MQ-135
void pageAir(int raw, int ppm)
{
  oled.clearDisplay();
  titleBar("AIR MQ-135");
  oled.setTextSize(1);
  oled.setCursor(0, 18);
  oled.print("PPM:");
  oled.setTextSize(2);
  oled.setCursor(34, 14);
  oled.print(ppm);
  drawBar(0, 32, 128, 7, ppm / 10);
  oled.setTextSize(1);
  oled.setCursor(0, 43);
  oled.print("RAW:");
  oled.print(raw);
  oled.setCursor(0, 55);
  oled.print("AIR:");
  oled.print(airLabel(ppm));
  oled.display();
}
// Page 2: Alcohol MQ-3
void pageAlc(int raw, int ppm)
{
  oled.clearDisplay();
  titleBar("ALCOHOL MQ-3");
  oled.setTextSize(1);
  oled.setCursor(0, 18);
  oled.print("PPM:");
  oled.setTextSize(2);
  oled.setCursor(34, 14);
  oled.print(ppm);
  drawBar(0, 32, 128, 7, ppm / 10);
  oled.setTextSize(1);
  oled.setCursor(0, 43);
  oled.print("RAW:");
  oled.print(raw);
  oled.setCursor(0, 55);
  oled.print(ppm >= 400 ? "!! ALCOHOL DETECTED" : "Clear");
  oled.display();
}
// Page 3: MQ-4 + MQ-7 analog
void pageGas(int ppm4, int ppm7)
{
  oled.clearDisplay();
  titleBar("GAS DETECT");
  bool g4 = ppm4 >= 300;
  bool g7 = ppm7 >= 200;
  oled.setTextSize(1);
  oled.setCursor(0, 18);
  oled.print("CH4:");
  oled.print(ppm4);
  oled.print(g4 ? " !" : " OK");
  drawBar(0, 26, 128, 6, constrain(ppm4 / 10, 0, 100));
  oled.setCursor(0, 36);
  oled.print("CO: ");
  oled.print(ppm7);
  oled.print(g7 ? " !" : " OK");
  drawBar(0, 44, 128, 6, constrain(ppm7 / 10, 0, 100));
  oled.setCursor(0, 55);
  oled.print("ADC max:4095 (12-bit)");
  oled.display();
}
// Page 4: Summary
void pageSummary(float t, float h, int a135, int a3, int a4, int a7)
{
  oled.clearDisplay();
  titleBar("SUMMARY");
  bool alert = a135 >= 500 || a3 >= 400 || a4 >= 300 || a7 >= 200 || t >= 35;
  oled.setTextSize(1);
  oled.setCursor(0, 18);
  oled.print("T:");
  oled.print(t, 1);
  oled.print(" H:");
  oled.print(h, 0);
  oled.print("%");
  oled.setCursor(0, 28);
  oled.print("AIR:");
  oled.print(airLabel(a135));
  oled.setCursor(0, 38);
  oled.print("ALC:");
  oled.print(a3 >= 400 ? "!" : "OK");
  oled.print(" CH4:");
  oled.print(a4 >= 300 ? "!" : "OK");
  oled.print(" CO:");
  oled.print(a7 >= 200 ? "!" : "OK");
  oled.setCursor(16, 52);
  oled.print(alert ? ">> ALERT <<" : "ALL NORMAL");
  oled.display();
}
void showAlert(int a135, int a3, int a4, int a7)
{
  oled.clearDisplay();
  oled.drawRect(0, 0, 128, 64, WHITE);
  oled.drawRect(2, 2, 124, 60, WHITE);
  oled.setTextSize(2);
  oled.setCursor(14, 6);
  oled.print("WARNING!");
  oled.setTextSize(1);
  int y = 30;
  if (a4 >= 300)
  {
    oled.setCursor(6, y);
    oled.print("GAS LEAK (CH4)");
    y += 10;
  }
  if (a7 >= 200)
  {
    oled.setCursor(6, y);
    oled.print("CO DETECTED");
    y += 10;
  }
  if (a3 >= 400)
  {
    oled.setCursor(6, y);
    oled.print("ALCOHOL HIGH");
    y += 10;
  }
  if (a135 >= 500)
  {
    oled.setCursor(6, y);
    oled.print("BAD AIR QUALITY");
  }
  oled.display();
}

// ── OLED helper: full-screen status message ─────────────
void oledMsg(const char *l1, const char *l2, const char *l3)
{
  oled.clearDisplay();
  oled.setTextColor(WHITE);
  oled.setTextSize(1);
  if (l1)
  {
    oled.setCursor(4, 6);
    oled.print(l1);
  }
  if (l2)
  {
    oled.setCursor(4, 26);
    oled.print(l2);
  }
  if (l3)
  {
    oled.setCursor(4, 46);
    oled.print(l3);
  }
  oled.display();
}

// Called by WiFiManager when it launches the config portal
void portalCallback(WiFiManager *wm)
{
  oledMsg("WiFi not set.", "Join hotspot:", AP_NAME);
  Serial.println("Config portal up. Join AP: " AP_NAME);
}

// ── Connect WiFi via captive portal ─────────────────────
void connectWiFi()
{
  WiFiManager wm;
  wm.setAPCallback(portalCallback);
  // After you tap Save in the portal, reboot for a clean
  // fresh connect (avoids sitting silently on the hotspot).
  wm.setSaveConfigCallback([]()
                           {
    Serial.println("WiFi saved -> restarting");
    delay(800);
    ESP.restart(); });
  // Keep the hotspot portal OPEN until a network is chosen
  // (0 = no timeout). This guarantees that if the saved WiFi
  // isn't available, the board waits on its own hotspot for
  // you to pick a new one — no button, no re-flash.
  wm.setConfigPortalTimeout(0);

  // OPTIONAL override: hold BOOT (GPIO 0) at power-up to wipe
  // the saved WiFi and force the portal even if the old
  // network would have connected. Not required for normal use.
  pinMode(PORTAL_BUTTON, INPUT_PULLUP);
  if (digitalRead(PORTAL_BUTTON) == LOW)
  {
    Serial.println("BOOT held -> wiping saved WiFi, forcing portal");
    wm.resetSettings();
  }

  oledMsg("Connecting to", "saved WiFi...", NULL);

  // autoConnect():
  //   1) tries the network saved from last time
  //   2) if it connects -> returns true immediately
  //   3) if that network is unavailable / none saved ->
  //      automatically starts the "ESP32-AirMonitor" hotspot
  //      portal and blocks here until you pick a network.
  bool ok = wm.autoConnect(AP_NAME, AP_PASS);

  wifiOK = ok && (WiFi.status() == WL_CONNECTED);
  if (wifiOK)
  {
    // Let the ESP32 silently rejoin this network if it drops
    WiFi.setAutoReconnect(true);
    WiFi.persistent(true);
    Serial.print("WiFi OK, IP: ");
    Serial.println(WiFi.localIP());
    oledMsg("WiFi connected!", WiFi.localIP().toString().c_str(), NULL);
  }
  else
  {
    Serial.println("WiFi not configured - running offline");
    oledMsg("WiFi not set.", "Running offline", "Sensors only");
  }
  delay(1200);
}

// ── Keep MQTT connected (non-blocking-ish, quick retries) ─
bool mqttEnsure()
{
  if (!wifiOK || WiFi.status() != WL_CONNECTED)
  {
    wifiOK = false;
    return false;
  }
  if (mqtt.connected())
    return true;

  Serial.print("MQTT connecting... ");
  int8_t ret;
  uint8_t tries = 3;
  while ((ret = mqtt.connect()) != 0 && tries--)
  {
    Serial.println(mqtt.connectErrorString(ret));
    mqtt.disconnect();
    delay(1000);
  }
  bool up = (ret == 0);
  Serial.println(up ? "MQTT connected" : "MQTT failed");
  return up;
}

// ── Publish all sensor values to Adafruit IO ────────────
void uploadToCloud(float t, float h, int a135, int a3, int a4, int a7)
{
  if (!mqttEnsure())
    return;
  if (!isnan(t))
    feedTemp.publish(t);
  if (!isnan(h))
    feedHum.publish(h);
  feedAir.publish((int32_t)a135);
  feedAlc.publish((int32_t)a3);
  feedCH4.publish((int32_t)a4);
  feedCO.publish((int32_t)a7);
  Serial.println("-> Uploaded to Adafruit IO");
}

void setup()
{
  Serial.begin(115200);
  dht.begin();
  Wire.begin(SDA_PIN, SCL_PIN);
  if (!oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR))
  {
    Serial.println("OLED fail!");
    while (1)
      ;
  }
  Serial.println("OLED success initiated!");

  oled.clearDisplay();
  oled.setTextColor(WHITE);
  oled.setTextSize(1);
  oled.setCursor(4, 4);
  oled.print("ESP32 Monitor");
  oled.setCursor(4, 18);
  oled.print("DHT+MQ3/4/7/135");
  oled.setCursor(4, 32);
  oled.print("WiFi + Adafruit IO");
  oled.setCursor(4, 46);
  oled.print("Warming up...");
  oled.display();
  delay(3000);

  // Bring up WiFi (captive portal on first use)
  connectWiFi();

  // MQ sensors need to warm up; do it now so first upload is valid
  oledMsg("Sensors warming", "up ~30s...", NULL);
  delay(30000);
}

void loop()
{
  float temp = dht.readTemperature();
  float hum = dht.readHumidity();
  float hi = dht.computeHeatIndex(temp, hum, false);

  // All 4 MQ sensors read directly
  int r135 = analogRead(MQ135_PIN);
  int r3 = analogRead(MQ3_PIN);
  int r4 = analogRead(MQ4_PIN);
  int r7 = analogRead(MQ7_PIN);
  int p135 = rawToPPM(r135, 1000);
  int p3 = rawToPPM(r3, 1000);
  int p4 = rawToPPM(r4, 1000);
  int p7 = rawToPPM(r7, 1000);

  Serial.printf("T:%.1f H:%.1f AIR:%d ALC:%d CH4:%d CO:%d\n",
                temp, hum, p135, p3, p4, p7);

  // Keep WiFi flag fresh; nudge a reconnect if it dropped.
  // (Non-blocking: WiFi.reconnect() just retries the SAVED
  // network in the background. To switch to a *different*
  // network, power-cycle so the portal can appear again.)
  wifiOK = (WiFi.status() == WL_CONNECTED);
  if (!wifiOK && (millis() - lastReconnect > 15000UL))
  {
    lastReconnect = millis();
    Serial.println("WiFi dropped -> reconnecting to saved network");
    WiFi.reconnect();
  }

  // ── Cloud upload on its own timer (runs even during alerts) ──
  if (millis() - lastUpload > UPLOAD_MS)
  {
    lastUpload = millis();
    uploadToCloud(temp, hum, p135, p3, p4, p7);
  }

  bool alert = r135 >= AIR_ALERT || r3 >= ALC_ALERT || r4 >= MQ4_ALERT || r7 >= MQ7_ALERT;
  if (alert)
  {
    alertFlip = !alertFlip;
    alertFlip ? showAlert(p135, p3, p4, p7)
              : pageSummary(temp, hum, p135, p3, p4, p7);
    delay(500);
    return;
  }

  if (millis() - lastPage > PAGE_MS)
  {
    page = (page + 1) % PAGES;
    lastPage = millis();
  }
  switch (page)
  {
  case 0:
    if (!isnan(temp))
      pageDHT(temp, hum, hi);
    break;
  case 1:
    pageAir(r135, p135);
    break;
  case 2:
    pageAlc(r3, p3);
    break;
  case 3:
    pageGas(p4, p7);
    break;
  case 4:
    pageSummary(temp, hum, p135, p3, p4, p7);
    break;
  }
  delay(1500);
}