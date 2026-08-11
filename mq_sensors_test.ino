/*  MQ SENSOR ADC DIAGNOSTIC — OLED output
   ─────────────────────────────────────────────────────
   Standalone. No WiFi, no MQTT, no DHT.
   Flash this to find out why all MQ sensors read 1000.
   ─────────────────────────────────────────────────────
   MQ-135  AOUT → GPIO 34      OLED SDA → GPIO 21
   MQ-3    AOUT → GPIO 35      OLED SCL → GPIO 22
   MQ-4    AOUT → GPIO 32
   MQ-7    AOUT → GPIO 33
   ─────────────────────────────────────────────────────
   Page 1 = live raw + voltage per sensor
   Page 2 = verdict per sensor (SAT / FLOAT / OK)
   Page 3 = min/max tracking (shows if values move at all)
   Pages auto-cycle every 4s.
   ─────────────────────────────────────────────────────
   READING THE VERDICT:
   SAT   raw >= 4090  -> divider missing/wrong, AOUT > 3.3V
   FLOAT raw <= 5     -> AOUT not connected
   LOW   raw < 100    -> divider ratio too aggressive
   OK    100-4089     -> working; expect 300-2500 clean air
   ───────────────────────────────────────────────────── */

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_W 128
#define SCREEN_H 64
#define OLED_ADDR 0x3C
#define SDA_PIN 21
#define SCL_PIN 22

#define MQ135_PIN 34
#define MQ3_PIN 35
#define MQ4_PIN 32
#define MQ7_PIN 33

#define SAMPLES 32     // oversampling per reading
#define PAGE_MS 4000UL // page auto-cycle
#define NSENS 4

Adafruit_SSD1306 oled(SCREEN_W, SCREEN_H, &Wire, -1);

const char *names[NSENS] = {"MQ135", "MQ3", "MQ4", "MQ7"};
const int pins[NSENS] = {MQ135_PIN, MQ3_PIN, MQ4_PIN, MQ7_PIN};

int raw[NSENS];
int rMin[NSENS], rMax[NSENS];

int page = 0;
unsigned long lastPage = 0;
unsigned long startMs = 0;

// ── Oversampled read ────────────────────────────────────
int readMQ(int pin)
{
  long sum = 0;
  for (int i = 0; i < SAMPLES; i++)
  {
    sum += analogRead(pin);
    delayMicroseconds(200);
  }
  return sum / SAMPLES;
}

// ── Verdict string per sensor ───────────────────────────
const char *verdict(int r)
{
  if (r >= 4090)
    return "SAT!";
  if (r <= 5)
    return "FLOAT";
  if (r < 100)
    return "LOW";
  return "OK";
}

void titleBar(const char *t)
{
  oled.setTextSize(1);
  oled.setTextColor(WHITE);
  oled.setCursor(0, 0);
  oled.print(t);
  for (int i = 0; i < 3; i++)
  {
    int dx = 108 + i * 7;
    i == page ? oled.fillCircle(dx, 3, 2, WHITE)
              : oled.drawCircle(dx, 3, 2, WHITE);
  }
  oled.drawFastHLine(0, 9, 128, WHITE);
}

// ── Page 0: raw value + voltage ─────────────────────────
void pageRaw()
{
  oled.clearDisplay();
  titleBar("ADC RAW / VOLTS");
  oled.setTextSize(1);
  for (int i = 0; i < NSENS; i++)
  {
    int y = 14 + i * 12;
    float v = raw[i] * 3.3f / 4095.0f;
    oled.setCursor(0, y);
    oled.print(names[i]);
    oled.setCursor(38, y);
    oled.print(raw[i]);
    oled.setCursor(72, y);
    oled.print(v, 2);
    oled.print("V");
    // bar showing position in full scale
    int w = map(raw[i], 0, 4095, 0, 18);
    oled.drawRect(108, y, 20, 7, WHITE);
    oled.fillRect(109, y + 1, w, 5, WHITE);
  }
  oled.display();
}

// ── Page 1: verdict ─────────────────────────────────────
void pageVerdict()
{
  oled.clearDisplay();
  titleBar("DIAGNOSIS");
  oled.setTextSize(1);
  int satCount = 0;
  for (int i = 0; i < NSENS; i++)
  {
    int y = 13 + i * 10;
    const char *v = verdict(raw[i]);
    if (raw[i] >= 4090)
      satCount++;
    oled.setCursor(0, y);
    oled.print(names[i]);
    oled.setCursor(38, y);
    oled.print(v);
    oled.setCursor(76, y);
    oled.print(raw[i]);
  }
  oled.drawFastHLine(0, 53, 128, WHITE);
  oled.setCursor(0, 56);
  if (satCount == NSENS)
    oled.print("ALL SAT: no divider!");
  else if (satCount > 0)
    oled.print("Check divider wiring");
  else
    oled.print("Dividers look OK");
  oled.display();
}

// ── Page 2: min/max drift tracking ──────────────────────
void pageMinMax()
{
  oled.clearDisplay();
  titleBar("MIN / MAX");
  oled.setTextSize(1);
  for (int i = 0; i < NSENS; i++)
  {
    int y = 13 + i * 10;
    oled.setCursor(0, y);
    oled.print(names[i]);
    oled.setCursor(38, y);
    oled.print(rMin[i]);
    oled.setCursor(70, y);
    oled.print("-");
    oled.setCursor(80, y);
    oled.print(rMax[i]);
  }
  oled.drawFastHLine(0, 53, 128, WHITE);
  oled.setCursor(0, 56);
  oled.print("Uptime: ");
  oled.print((millis() - startMs) / 1000);
  oled.print("s");
  oled.display();
}

void setup()
{
  Serial.begin(115200);
  delay(300);

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db); // full 0-3.3V range

  Wire.begin(SDA_PIN, SCL_PIN);
  if (!oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR))
  {
    Serial.println("OLED fail!");
    while (1)
      ;
  }

  for (int i = 0; i < NSENS; i++)
  {
    rMin[i] = 4095;
    rMax[i] = 0;
  }

  oled.clearDisplay();
  oled.setTextColor(WHITE);
  oled.setTextSize(1);
  oled.setCursor(2, 4);
  oled.print("MQ ADC DIAGNOSTIC");
  oled.setCursor(2, 20);
  oled.print("4095 = saturated");
  oled.setCursor(2, 32);
  oled.print("Need 10k/20k divider");
  oled.setCursor(2, 44);
  oled.print("on every AOUT wire");
  oled.display();
  delay(3000);

  startMs = millis();
  lastPage = millis();
}

void loop()
{
  for (int i = 0; i < NSENS; i++)
  {
    raw[i] = readMQ(pins[i]);
    if (raw[i] < rMin[i])
      rMin[i] = raw[i];
    if (raw[i] > rMax[i])
      rMax[i] = raw[i];
  }

  // Mirror to Serial as well, in case you want a log
  Serial.printf("MQ135:%4d(%.2fV) MQ3:%4d(%.2fV) MQ4:%4d(%.2fV) MQ7:%4d(%.2fV)\n",
                raw[0], raw[0] * 3.3f / 4095.0f,
                raw[1], raw[1] * 3.3f / 4095.0f,
                raw[2], raw[2] * 3.3f / 4095.0f,
                raw[3], raw[3] * 3.3f / 4095.0f);

  if (millis() - lastPage > PAGE_MS)
  {
    page = (page + 1) % 3;
    lastPage = millis();
  }

  switch (page)
  {
  case 0:
    pageRaw();
    break;
  case 1:
    pageVerdict();
    break;
  case 2:
    pageMinMax();
    break;
  }
  delay(500);
}