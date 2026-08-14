/* ==================================================================
   ESP32 Air Monitor — FIXED v2
   ==================================================================
   WHY THE ORIGINAL ALWAYS SHOWED "CO DETECTED"
   ------------------------------------------------------------------
   1) THE THRESHOLD WAS BELOW THE SENSOR'S CLEAN-AIR BASELINE.
      An MQ-7 sitting in clean air, after warm-up, puts roughly
      1.0-2.0 V on AOUT.  Through your 10k/20k divider that is
      0.7-1.3 V at the pin  ->  raw ADC ~= 850-1650.
      Your trigger was  r7 >= 800.  So a perfectly clean room
      already exceeded it.  MQ-7 also had the LOWEST threshold of the
      four sensors, which is why CO — and only CO — fired every time.

   2) rawToPPM() IS NOT PPM.  map(raw,0,4095,0,1000) is a straight
      line from the ADC count.  MQ sensors are logarithmic in
      Rs/R0 and every individual sensor has a different R0.  A raw
      count of 819 became "200 ppm CO" out of thin air.

   3) TWO DIFFERENT THRESHOLD SETS.  loop() decided "alert" from RAW
      values (MQ7_ALERT 800) but showAlert() decided WHICH LINE to
      print from PPM values (a7 >= 200, i.e. raw >= 819).  They
      disagree, so you could get a WARNING box with no reason listed,
      or a reason that wasn't what triggered it.

   4) NO WARM-UP GATE.  The old sketch waited 3 s + 30 s then went
      straight to alerting.  A cold MQ element reads HIGH while the
      heater stabilises, so the first minutes are guaranteed alarms.

   5) NO SANITY CHECK.  If MQ-7's AOUT wire is loose, GPIO33 floats
      and reads random noise — which also looks like "CO".

   THE FIX
   ------------------------------------------------------------------
   * Each sensor is calibrated at boot: after preheat, the clean-air
     resistance is measured and R0 is derived.  Alerts are then based
     on Rs/R0 DROPPING below a per-sensor fraction — self-calibrating,
     so it no longer matters what your baseline voltage happens to be.
   * ONE set of flags (s[i].alarm) drives the trigger, the WARNING
     screen, the summary page and the serial log.  No more mismatch.
   * 16-sample averaged reads, 3-strike debounce + hysteresis.
   * Disconnected / rail-stuck sensors are detected and excluded.
   * ppm is still shown but clearly marked "est" — it is only
     meaningful after you calibrate the curve for your own sensor.
   ================================================================== */

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <DHT.h>
#include <WiFi.h>
#include <WiFiManager.h>          // tzapu/WiFiManager
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include <math.h>
#include <string.h>

/* ---------------- Adafruit IO (leave blank to run offline) -------- */
#define AIO_USERNAME   ""
#define AIO_KEY        ""
#define AIO_SERVER     "io.adafruit.com"
#define AIO_SERVERPORT 1883

#define AP_NAME       "ESP32-AirMonitor"
#define AP_PASS       "12345678"
#define PORTAL_BUTTON 0
#define UPLOAD_MS     20000UL

/* ---------------- Display / DHT ----------------------------------- */
#define SCREEN_W  128
#define SCREEN_H   64
#define OLED_ADDR 0x3C
#define SDA_PIN    21
#define SCL_PIN    22

#define DHT_PIN    23
#define DHT_TYPE   DHT11   // <-- your header comment says DHT22.
                           //     DHT11 and DHT22 are NOT interchangeable.
                           //     Set this to whatever is physically wired.

/* ---------------- MQ analog pins (all ADC1 - safe with WiFi) ------ */
#define MQ135_PIN 34
#define MQ3_PIN   35
#define MQ4_PIN   32
#define MQ7_PIN   33

/* ---------------- Front-end electrical model ----------------------
   Vout(sensor) = Vadc * DIV_RATIO
   10k in series + 20k to GND  ->  Vadc = Vout * 20/30  ->  ratio 1.5
   If you used a different divider, fix DIV_RATIO or readings skew.  */
#define ADC_MAX    4095.0f
#define VREF       3.30f    // full-scale volts at 11 dB attenuation
#define VC         5.00f    // supply feeding the MQ sensing circuit
#define DIV_RATIO  1.5f
#define RL_KOHM    10.0f    // load resistor ON THE MQ MODULE (check yours!)

/* ---------------- Timing / robustness ----------------------------- */
#define PREHEAT_MS      90000UL  // MQ heater stabilise before calibrating
#define ADC_SAMPLES     16
#define RAW_MIN_VALID   25       // below this = wire off / shorted low
#define RAW_MAX_VALID   4070     // at this = no divider / shorted high
#define ALARM_STRIKES   3        // consecutive reads before alarming
#define CLEAR_MARGIN    1.15f    // hysteresis on the way back down

#define PAGE_MS 3000
#define PAGES      5

Adafruit_SSD1306 oled(SCREEN_W, SCREEN_H, &Wire, -1);
DHT dht(DHT_PIN, DHT_TYPE);

/* ================= sensor model =================================== */
enum { S_AIR = 0, S_ALC, S_CH4, S_CO, S_COUNT };

struct MQSensor {
  const char *name;      // short label for the OLED
  const char *gas;
  uint8_t     pin;
  float       cleanRatio;// datasheet Rs/R0 in clean air
  float       curveA;    // ppm = A * (Rs/R0)^B   (rough, per datasheet)
  float       curveB;
  float       alertRatio;// alarm when Rs/R0 falls to/below this

  /* runtime */
  int    raw;
  float  rs, r0, ratio, ppm;
  bool   ok;             // wiring looks sane
  bool   alarm;
  uint8_t strikes;
};

/* alertRatio 0.50 == "Rs halved vs. clean air" == a real gas event.
   Lower the number to make a sensor LESS sensitive, raise it to make
   it MORE sensitive.  Tune these against a known source, not guesses. */
MQSensor s[S_COUNT] = {
  /* name        gas        pin        clean   A          B        alert */
  { "AIR", "MQ-135",  MQ135_PIN,  3.60f, 116.602f, -2.769f, 0.45f, 0,0,0,0,0,false,false,0 },
  { "ALC", "MQ-3",    MQ3_PIN,   60.00f,   0.409f, -1.492f, 0.30f, 0,0,0,0,0,false,false,0 },
  { "CH4", "MQ-4",    MQ4_PIN,    4.40f,1012.700f, -2.786f, 0.50f, 0,0,0,0,0,false,false,0 },
  { "CO",  "MQ-7",    MQ7_PIN,   27.50f,  99.042f, -1.518f, 0.50f, 0,0,0,0,0,false,false,0 },
};

/* ================= state ========================================== */
int  page = 0;
unsigned long lastPage = 0;
bool alertFlip = false;

unsigned long lastUpload    = 0;
unsigned long lastReconnect = 0;
unsigned long lastMqttTry   = 0;
bool wifiOK      = false;
bool calibrated  = false;

/* ================= MQTT =========================================== */
WiFiClient netClient;
Adafruit_MQTT_Client mqtt(&netClient, AIO_SERVER, AIO_SERVERPORT,
                          AIO_USERNAME, AIO_KEY);

Adafruit_MQTT_Publish feedTemp = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/temperature");
Adafruit_MQTT_Publish feedHum  = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/humidity");
Adafruit_MQTT_Publish feedAir  = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/air-quality");
Adafruit_MQTT_Publish feedAlc  = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/alcohol");
Adafruit_MQTT_Publish feedCH4  = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/methane");
Adafruit_MQTT_Publish feedCO   = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/carbon-monoxide");

static inline bool cloudConfigured() { return strlen(AIO_USERNAME) > 0 && strlen(AIO_KEY) > 0; }

/* ================= ADC helpers ==================================== */
int readRawAvg(uint8_t pin) {
  long acc = 0;
  analogRead(pin);                      // throw the first one away
  for (int i = 0; i < ADC_SAMPLES; i++) {
    acc += analogRead(pin);
    delayMicroseconds(300);
  }
  return (int)(acc / ADC_SAMPLES);
}

/* Sensor resistance from the raw ADC count. */
float rsFromRaw(int raw) {
  float vadc = (raw / ADC_MAX) * VREF;
  float vout = vadc * DIV_RATIO;
  if (vout < 0.05f)      vout = 0.05f;
  if (vout > VC - 0.05f) vout = VC - 0.05f;
  return RL_KOHM * (VC - vout) / vout;   // kOhm
}

/* ================= calibration ==================================== */
/* Must be run in CLEAN AIR.  Derives R0 for each sensor so that the
   alarm logic is relative to THIS sensor in THIS room — the whole
   reason the old fixed thresholds misfired.                          */
void calibrateSensors() {
  Serial.println(F("--- clean-air calibration ---"));
  for (int i = 0; i < S_COUNT; i++) {
    long acc = 0;
    for (int n = 0; n < 20; n++) { acc += readRawAvg(s[i].pin); delay(50); }
    int raw = (int)(acc / 20);
    s[i].raw = raw;
    s[i].ok  = (raw >= RAW_MIN_VALID && raw <= RAW_MAX_VALID);

    if (s[i].ok) {
      s[i].rs = rsFromRaw(raw);
      s[i].r0 = s[i].rs / s[i].cleanRatio;
      Serial.printf("%-3s %-7s raw=%4d  Rs=%8.2fk  R0=%8.2fk  OK\n",
                    s[i].name, s[i].gas, raw, s[i].rs, s[i].r0);
    } else {
      s[i].r0 = 0;
      Serial.printf("%-3s %-7s raw=%4d  *** WIRING FAULT - disabled ***\n",
                    s[i].name, s[i].gas, raw);
      Serial.println(F("    raw ~0   = AOUT not connected / divider shorted to GND"));
      Serial.println(F("    raw 4095 = no voltage divider, or AOUT shorted to 5V"));
    }
  }
  calibrated = true;
  Serial.println(F("-----------------------------"));
}

/* ================= per-cycle sensor update ======================== */
void updateSensors() {
  for (int i = 0; i < S_COUNT; i++) {
    MQSensor &m = s[i];
    if (!m.ok) { m.alarm = false; m.ratio = 0; m.ppm = 0; continue; }

    m.raw = readRawAvg(m.pin);

    /* wire fell off while running? */
    if (m.raw < RAW_MIN_VALID) { m.ok = false; m.alarm = false; continue; }

    m.rs    = rsFromRaw(m.raw);
    m.ratio = (m.r0 > 0.001f) ? (m.rs / m.r0) : 1.0f;

    /* informational only — needs a real calibration to be trusted */
    float p = m.curveA * powf(m.ratio > 0.001f ? m.ratio : 0.001f, m.curveB);
    if (!isfinite(p)) p = 0;
    m.ppm = constrain(p, 0.0f, 10000.0f);

    /* debounce + hysteresis so one noisy sample cannot latch an alarm */
    if (m.ratio <= m.alertRatio) {
      if (m.strikes < 250) m.strikes++;
    } else if (m.ratio > m.alertRatio * CLEAR_MARGIN) {
      m.strikes = 0;
    }
    m.alarm = (m.strikes >= ALARM_STRIKES);
  }
}

bool anyAlarm() {
  for (int i = 0; i < S_COUNT; i++) if (s[i].alarm) return true;
  return false;
}

/* ================= UI ============================================= */
const char *airLabel(float ratio) {
  if (ratio >= 0.90f) return "GOOD";
  if (ratio >= 0.70f) return "MODERATE";
  if (ratio >= 0.55f) return "POOR";
  if (ratio >= 0.40f) return "BAD";
  return                     "HAZARD";
}

/* 0 % = clean air, 100 % = at the alarm point */
int gasPct(const MQSensor &m) {
  if (!m.ok || m.ratio <= 0) return 0;
  float span = 1.0f - m.alertRatio;
  if (span < 0.01f) span = 0.01f;
  return (int)constrain((1.0f - m.ratio) / span * 100.0f, 0.0f, 100.0f);
}

void drawBar(int x, int y, int w, int h, int pct) {
  oled.drawRect(x, y, w, h, WHITE);
  oled.fillRect(x + 1, y + 1, (w - 2) * constrain(pct, 0, 100) / 100, h - 2, WHITE);
}

void titleBar(const char *t) {
  oled.setTextSize(1);
  oled.setTextColor(WHITE);
  oled.setCursor(0, 4);
  oled.print(t);
  wifiOK ? oled.fillCircle(122, 7, 3, WHITE) : oled.drawCircle(122, 7, 3, WHITE);
  for (int i = 0; i < PAGES; i++) {
    int dx = 76 + i * 6;
    i == page ? oled.fillCircle(dx, 7, 2, WHITE) : oled.drawCircle(dx, 7, 2, WHITE);
  }
}

void printF(float v, int dec) { isnan(v) ? (void)oled.print("--") : (void)oled.print(v, dec); }

void oledMsg(const char *l1, const char *l2, const char *l3) {
  oled.clearDisplay();
  oled.setTextColor(WHITE);
  oled.setTextSize(1);
  if (l1) { oled.setCursor(4, 6);  oled.print(l1); }
  if (l2) { oled.setCursor(4, 26); oled.print(l2); }
  if (l3) { oled.setCursor(4, 46); oled.print(l3); }
  oled.display();
}

/* Page 0 — temp & humidity */
void pageDHT(float t, float h, float hi) {
  oled.clearDisplay();
  titleBar("TEMP & HUM");
  oled.setTextSize(1); oled.setCursor(0, 18); oled.print("T:");
  oled.setTextSize(2); oled.setCursor(16, 14); printF(t, 1); oled.print("C");
  drawBar(0, 30, 128, 6, isnan(t) ? 0 : (int)constrain(t / 50 * 100, 0.0f, 100.0f));
  oled.setTextSize(1);
  oled.setCursor(0, 40); oled.print("H:"); printF(h, 1); oled.print("%");
  drawBar(0, 50, 128, 6, isnan(h) ? 0 : (int)h);
  oled.setCursor(70, 40); oled.print("HI:"); printF(hi, 1);
  oled.display();
}

/* Generic single-sensor page */
void pageSensor(const char *title, const MQSensor &m, bool showAirLabel) {
  oled.clearDisplay();
  titleBar(title);
  oled.setTextSize(1);
  if (!m.ok) {
    oled.setCursor(0, 26); oled.print("SENSOR NOT CONNECTED");
    oled.setCursor(0, 40); oled.print("check AOUT + divider");
    oled.display();
    return;
  }
  oled.setCursor(0, 18); oled.print("Rs/R0:");
  oled.setTextSize(2);   oled.setCursor(44, 14); oled.print(m.ratio, 2);
  drawBar(0, 32, 128, 7, gasPct(m));
  oled.setTextSize(1);
  oled.setCursor(0, 43);
  oled.print("raw:"); oled.print(m.raw);
  oled.print("  est:"); oled.print((int)m.ppm);
  oled.setCursor(0, 55);
  if (showAirLabel) { oled.print("AIR:"); oled.print(airLabel(m.ratio)); }
  else              { oled.print(m.alarm ? "!! DETECTED" : "Clear"); }
  oled.display();
}

/* Page 3 — CH4 + CO together */
void pageGas() {
  oled.clearDisplay();
  titleBar("GAS DETECT");
  oled.setTextSize(1);
  oled.setCursor(0, 18);
  oled.print("CH4 ");
  s[S_CH4].ok ? (void)oled.print(s[S_CH4].ratio, 2) : (void)oled.print("N/C");
  oled.print(s[S_CH4].alarm ? " !" : " OK");
  drawBar(0, 26, 128, 6, gasPct(s[S_CH4]));
  oled.setCursor(0, 36);
  oled.print("CO  ");
  s[S_CO].ok ? (void)oled.print(s[S_CO].ratio, 2) : (void)oled.print("N/C");
  oled.print(s[S_CO].alarm ? " !" : " OK");
  drawBar(0, 44, 128, 6, gasPct(s[S_CO]));
  oled.setCursor(0, 55);
  oled.print("Rs/R0 low = gas");
  oled.display();
}

/* Page 4 — summary */
void pageSummary(float t, float h) {
  oled.clearDisplay();
  titleBar("SUMMARY");
  bool alert = anyAlarm() || (!isnan(t) && t >= 35);
  oled.setTextSize(1);
  oled.setCursor(0, 18);
  oled.print("T:"); printF(t, 1); oled.print(" H:"); printF(h, 0); oled.print("%");
  oled.setCursor(0, 28);
  oled.print("AIR:");
  s[S_AIR].ok ? (void)oled.print(airLabel(s[S_AIR].ratio)) : (void)oled.print("N/C");
  oled.setCursor(0, 38);
  for (int i = 1; i < S_COUNT; i++) {
    oled.print(s[i].name); oled.print(":");
    oled.print(!s[i].ok ? "-" : (s[i].alarm ? "!" : "OK"));
    oled.print(" ");
  }
  oled.setCursor(16, 52);
  oled.print(alert ? ">> ALERT <<" : "ALL NORMAL");
  oled.display();
}

/* WARNING screen — driven by the SAME flags as the trigger */
void showAlert() {
  oled.clearDisplay();
  oled.drawRect(0, 0, 128, 64, WHITE);
  oled.drawRect(2, 2, 124, 60, WHITE);
  oled.setTextSize(2); oled.setCursor(14, 6); oled.print("WARNING!");
  oled.setTextSize(1);
  int y = 30;
  if (s[S_CH4].alarm) { oled.setCursor(6, y); oled.print("GAS LEAK (CH4)");   y += 10; }
  if (s[S_CO ].alarm) { oled.setCursor(6, y); oled.print("CO DETECTED");      y += 10; }
  if (s[S_ALC].alarm) { oled.setCursor(6, y); oled.print("ALCOHOL HIGH");     y += 10; }
  if (s[S_AIR].alarm) { oled.setCursor(6, y); oled.print("BAD AIR QUALITY");  y += 10; }
  oled.display();
}

/* ================= WiFi =========================================== */
void portalCallback(WiFiManager *wm) {
  oledMsg("WiFi not set.", "Join hotspot:", AP_NAME);
  Serial.println(F("Config portal up. Join AP: " AP_NAME));
}

void connectWiFi() {
  WiFiManager wm;
  wm.setAPCallback(portalCallback);
  wm.setSaveConfigCallback([]() {
    Serial.println(F("WiFi saved -> restarting"));
    delay(800);
    ESP.restart();
  });
  wm.setConfigPortalTimeout(0);

  pinMode(PORTAL_BUTTON, INPUT_PULLUP);
  if (digitalRead(PORTAL_BUTTON) == LOW) {
    Serial.println(F("BOOT held -> wiping saved WiFi, forcing portal"));
    wm.resetSettings();
  }

  oledMsg("Connecting to", "saved WiFi...", NULL);
  bool ok = wm.autoConnect(AP_NAME, AP_PASS);

  wifiOK = ok && (WiFi.status() == WL_CONNECTED);
  if (wifiOK) {
    WiFi.setAutoReconnect(true);
    WiFi.persistent(true);
    Serial.print(F("WiFi OK, IP: ")); Serial.println(WiFi.localIP());
    oledMsg("WiFi connected!", WiFi.localIP().toString().c_str(), NULL);
  } else {
    Serial.println(F("WiFi not configured - running offline"));
    oledMsg("WiFi not set.", "Running offline", "Sensors only");
  }
  delay(1200);
}

/* ================= MQTT =========================================== */
bool mqttEnsure() {
  if (!cloudConfigured()) return false;
  if (!wifiOK || WiFi.status() != WL_CONNECTED) { wifiOK = false; return false; }
  if (mqtt.connected()) return true;
  if (millis() - lastMqttTry < 10000UL) return false;   // don't hammer the broker
  lastMqttTry = millis();

  Serial.print(F("MQTT connecting... "));
  int8_t ret = mqtt.connect();
  if (ret != 0) {
    Serial.println(mqtt.connectErrorString(ret));
    mqtt.disconnect();
    return false;
  }
  Serial.println(F("MQTT connected"));
  return true;
}

void uploadToCloud(float t, float h) {
  if (!mqttEnsure()) return;
  if (!isnan(t)) feedTemp.publish(t);
  if (!isnan(h)) feedHum.publish(h);
  if (s[S_AIR].ok) feedAir.publish(s[S_AIR].ppm);
  if (s[S_ALC].ok) feedAlc.publish(s[S_ALC].ppm);
  if (s[S_CH4].ok) feedCH4.publish(s[S_CH4].ppm);
  if (s[S_CO ].ok) feedCO .publish(s[S_CO ].ppm);
  Serial.println(F("-> Uploaded to Adafruit IO"));
}

/* ================= setup ========================================== */
void setup() {
  Serial.begin(115200);
  delay(200);
  dht.begin();

  analogReadResolution(12);
  for (int i = 0; i < S_COUNT; i++) {
    pinMode(s[i].pin, INPUT);
    analogSetPinAttenuation(s[i].pin, ADC_11db);   // full 0-3.3 V range
  }

  Wire.begin(SDA_PIN, SCL_PIN);
  if (!oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println(F("OLED fail!"));
    while (1) delay(100);                 // keep the watchdog fed
  }
  Serial.println(F("OLED success initiated!"));

  oledMsg("ESP32 Air Monitor", "DHT + MQ 3/4/7/135", "starting...");
  delay(1500);

  connectWiFi();

  /* Preheat FIRST, calibrate SECOND, alarm THIRD.
     Do this with the board in clean air — that is what R0 means. */
  unsigned long t0 = millis();
  while (millis() - t0 < PREHEAT_MS) {
    unsigned long left = (PREHEAT_MS - (millis() - t0)) / 1000;
    char l2[24]; snprintf(l2, sizeof(l2), "%lus left", left);
    oledMsg("Warming up MQ", l2, "keep air CLEAN");
    delay(500);
  }

  oledMsg("Calibrating", "clean-air R0...", NULL);
  calibrateSensors();

  bool anyOk = false;
  for (int i = 0; i < S_COUNT; i++) if (s[i].ok) anyOk = true;
  if (!anyOk) oledMsg("No MQ sensor", "responded.", "Check wiring!");
  else        oledMsg("Calibrated.", "Monitoring...", NULL);
  delay(1500);
  lastPage = millis();
}

/* ================= loop =========================================== */
void loop() {
  float temp = dht.readTemperature();
  float hum  = dht.readHumidity();
  float hi   = (isnan(temp) || isnan(hum)) ? NAN
                                           : dht.computeHeatIndex(temp, hum, false);

  updateSensors();

  Serial.printf("T:%.1f H:%.1f | AIR %.2f%s | ALC %.2f%s | CH4 %.2f%s | CO %.2f%s\n",
                temp, hum,
                s[S_AIR].ratio, s[S_AIR].alarm ? "!" : "",
                s[S_ALC].ratio, s[S_ALC].alarm ? "!" : "",
                s[S_CH4].ratio, s[S_CH4].alarm ? "!" : "",
                s[S_CO ].ratio, s[S_CO ].alarm ? "!" : "");

  wifiOK = (WiFi.status() == WL_CONNECTED);
  if (!wifiOK && (millis() - lastReconnect > 15000UL)) {
    lastReconnect = millis();
    Serial.println(F("WiFi dropped -> reconnecting to saved network"));
    WiFi.reconnect();
  }

  if (millis() - lastUpload > UPLOAD_MS) {
    lastUpload = millis();
    uploadToCloud(temp, hum);
  }

  if (anyAlarm()) {
    alertFlip = !alertFlip;
    alertFlip ? showAlert() : pageSummary(temp, hum);
    delay(700);
    return;
  }

  if (millis() - lastPage > PAGE_MS) {
    page = (page + 1) % PAGES;
    lastPage = millis();
  }
  switch (page) {
    case 0: pageDHT(temp, hum, hi);                    break;
    case 1: pageSensor("AIR MQ-135",   s[S_AIR], true); break;
    case 2: pageSensor("ALCOHOL MQ-3", s[S_ALC], false);break;
    case 3: pageGas();                                 break;
    case 4: pageSummary(temp, hum);                    break;
  }
  delay(1000);
}