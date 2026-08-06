/*  ESP32 DevKit — 5 Sensor Monitor
   ─────────────────────────────────────
   ESP32 has 18 analog pins — NO MUX!
   All MQ sensors use AOUT directly.
   ─────────────────────────────────────
   DHT22   SIG  → GPIO 4
   MQ-135  AOUT → GPIO 34 (input only)
   MQ-3    AOUT → GPIO 35 (input only)
   MQ-4    AOUT → GPIO 32
   MQ-7    AOUT → GPIO 33
   OLED    SDA  → GPIO 21
   OLED    SCL  → GPIO 22
   ─────────────────────────────────────
   ADC: 12-bit (0–4095) @ 3.3V max
   IMPORTANT: Add 10kΩ/20kΩ voltage
   divider on every MQ AOUT wire!
   MQ outputs up to 5V → ESP32 max 3.3V
   ─────────────────────────────────────
   Libraries:
   • Adafruit SSD1306 + GFX
   • DHT sensor library
   • Adafruit Unified Sensor             */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <DHT.h>

#define SCREEN_W 128
#define SCREEN_H 64
#define OLED_ADDR 0x3C
#define SDA_PIN 21
#define SCL_PIN 22

#define DHT_PIN 23
#define DHT_TYPE DHT11

// All MQ sensors → direct AOUT
// (via 10kΩ/20kΩ voltage divider!)
#define MQ135_PIN 33 // 34
#define MQ3_PIN 32   // 35
#define MQ2_PIN 35   // 32
#define MQ7_PIN 34   // 33

// ESP32 ADC: 12-bit = 0-4095
#define ADC_MAX 4095
#define AIR_ALERT 2000
#define ALC_ALERT 1600
#define MQ2_ALERT 1200
#define MQ7_ALERT 1800

Adafruit_SSD1306 oled(
    SCREEN_W, SCREEN_H, &Wire, -1);
DHT dht(DHT_PIN, DHT_TYPE);

int page = 0;
unsigned long lastPage = 0;
#define PAGE_MS 3000
#define PAGES 5
bool alertFlip = false;

int rawToPPM(int raw, int maxPPM)
{
  return map(raw, 0, ADC_MAX,
             0, maxPPM);
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

void drawBar(int x, int y,
             int w, int h, int pct)
{
  oled.drawRect(x, y, w, h, WHITE);
  oled.fillRect(x + 1, y + 1,
                (w - 2) * constrain(pct, 0, 100) / 100,
                h - 2, WHITE);
}

void titleBar(const char *t)
{
  oled.setTextSize(1);
  oled.setTextColor(WHITE);
  oled.setCursor(0, 4);
  oled.print(t);
  for (int i = 0; i < PAGES; i++)
  {
    int dx = 99 + i * 6;
    i == page
        ? oled.fillCircle(dx, 7, 2, WHITE)
        : oled.drawCircle(dx, 7, 2, WHITE);
  }
}

// Page 0: Temp & Humidity
void pageDHT(float t, float h,
             float hi)
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
  drawBar(0, 30, 128, 6,
          constrain(t / 50 * 100, 0, 100));
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
void pageGas(int ppm2, int ppm7)
{
  oled.clearDisplay();
  titleBar("GAS DETECT");
  bool g2 = ppm2 >= 300;
  bool g7 = ppm7 >= 400;
  oled.setTextSize(1);
  oled.setCursor(0, 18);
  oled.print("LPG:");
  oled.print(ppm2);
  oled.print(g2 ? " !" : " OK");
  drawBar(0, 26, 128, 6,
          constrain(ppm2 / 10, 0, 100));
  oled.setCursor(0, 36);
  oled.print("CO: ");
  oled.print(ppm7);
  oled.print(g7 ? " !" : " OK");
  drawBar(0, 44, 128, 6,
          constrain(ppm7 / 10, 0, 100));
  oled.setCursor(0, 55);
  oled.print("ADC max:4095 (12-bit)");
  oled.display();
}

// Page 4: Summary
void pageSummary(
    float t, float h,
    int a135, int a3,
    int a2, int a7)
{
  oled.clearDisplay();
  titleBar("SUMMARY");
  bool alert =
      a135 >= 500 || a3 >= 400 ||
      a2 >= 300 || a7 >= 400 ||
      t >= 35;
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
  oled.print(" LPG:");
  oled.print(a2 >= 300 ? "!" : "OK");
  oled.print(" CO:");
  oled.print(a7 >= 500 ? "!" : "OK");
  oled.setCursor(16, 52);
  oled.print(alert ? ">> ALERT <<" : "ALL NORMAL");
  oled.display();
}

void showAlert(
    int a135, int a3,
    int a2, int a7)
{
  oled.clearDisplay();
  oled.drawRect(0, 0, 128, 64, WHITE);
  oled.drawRect(2, 2, 124, 60, WHITE);
  oled.setTextSize(2);
  oled.setCursor(14, 6);
  oled.print("WARNING!");
  oled.setTextSize(1);
  int y = 30;
  if (a2 >= 300)
  {
    oled.setCursor(6, y);
    oled.print("GAS LEAK (LPG)");
    y += 10;
  }
  if (a7 >= 400)
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

void setup()
{
  Serial.begin(115200);
  dht.begin();
  Wire.begin(SDA_PIN, SCL_PIN);

  if (!oled.begin(
          SSD1306_SWITCHCAPVCC, OLED_ADDR))
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
  oled.print("DHT11+MQ3+2+7+135");
  oled.setCursor(4, 32);
  oled.print("No MUX needed!");
  oled.setCursor(4, 46);
  oled.print("Warming up 60s...");
  oled.display();
  delay(5000);
}

void loop()
{
  float temp = dht.readTemperature();
  float hum = dht.readHumidity();
  float hi = dht.computeHeatIndex(temp, hum, false);

  // All 4 MQ sensors read directly
  int r135 = analogRead(MQ135_PIN);
  int r3 = analogRead(MQ3_PIN);
  int r2 = analogRead(MQ2_PIN);
  int r7 = analogRead(MQ7_PIN);

  int p135 = rawToPPM(r135, 1000);
  int p3 = rawToPPM(r3, 1000);
  int p2 = rawToPPM(r2, 1000);
  int p7 = rawToPPM(r7, 1000);

  Serial.printf(
      "T:%.1f H:%.1f AIR:%d ALC:%d LPG:%d CO:%d\n",
      temp, hum, p135, p3, p2, p7);

  bool alert =
      r135 >= AIR_ALERT || r3 >= ALC_ALERT ||
      r2 >= MQ2_ALERT || r7 >= MQ7_ALERT;

  if (alert)
  {
    alertFlip = !alertFlip;
    alertFlip
        ? showAlert(p135, p3, p2, p7)
        : pageSummary(temp, hum,
                      p135, p3, p2, p7);
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
    pageGas(p2, p7);
    break;
  case 4:
    pageSummary(temp, hum,
                p135, p3, p2, p7);
    break;
  }
  delay(2000);
}
