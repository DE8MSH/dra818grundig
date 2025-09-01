#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <SoftwareSerial.h>
#include "logo_128x32.h"   // ← Bootlogo (128x32, MSB-first für drawBitmap)

// ------------------- Display -------------------
#define OLED_WIDTH  128
#define OLED_HEIGHT  32
#define OLED_ADDR   0x3C
Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, -1);

// ------------------- Pins -------------------
const uint8_t PIN_I2C_SDA  = 4;   // GP4
const uint8_t PIN_I2C_SCL  = 5;   // GP5
const uint8_t PIN_POT_SQL  = 26;  // ADC0 -> Squelch (0..9)
const uint8_t PIN_POT_VOL  = 27;  // ADC1 -> Volume  (0..7 -> DRA 1..8)
const uint8_t PIN_SW_PLUS  = 14;  // Drehschalter "+"
const uint8_t PIN_SW_MINUS = 15;  // Drehschalter "-"

// DRA818 SoftwareSerial auf GP2/GP3
const uint8_t PIN_DRA_RX = 3;     // Pico RX  (vom DRA TXD)
const uint8_t PIN_DRA_TX = 2;     // Pico TX  (zum DRA RXD)
SoftwareSerial DRA(PIN_DRA_RX, PIN_DRA_TX); // (RX, TX)

// optional:
const int PIN_DRA_PD  = 16;       // PowerDown: HIGH=aktiv
const int PIN_DRA_PTT = 17;       // PTT: HIGH=RX, LOW=TX

// ------------------- Frequenzen -------------------
const char* FREQS[] = { "144.5250", "144.8000", "145.3000", "145.3250", "145.3500" };
const int   FREQS_N  = sizeof(FREQS)/sizeof(FREQS[0]);

// ------------------- Zustände -------------------
int idxFreq = 0;                 // Start: 144.5250
int volStep = -1;                // 0..7 (→ DRA 1..8)
int sqlStep = -1;                // 0..9
bool lastPlus=false, lastMinus=false;
unsigned long lastStepMs=0;

// ------------------- Helpers -------------------
int adcToSteps(int raw, int steps) {
  int maxv = (raw > 1023 && raw <= 4095) ? 4095 : 1023;
  int s = map(raw, 0, maxv, 0, steps-1);
  if (s < 0) s = 0; if (s > steps-1) s = steps-1;
  return s;
}

void draSend(const String& cmd) {
  DRA.print(cmd); DRA.print("\r\n");
  unsigned long t0 = millis();
  while (millis() - t0 < 60) { while (DRA.available()) DRA.read(); } // RX-Puffer leeren
}

// AT+DMOSETGROUP=GBW,TFV,RFV,TxCTCSS,SQ,RxCTCSS  | GBW: 0=12.5kHz, 1=25kHz
void draSetGroup(const char* mhz, int sq /*0..9*/) {
  if (sq < 0) sq = 0; if (sq > 9) sq = 9;          // ggf. auf 8 begrenzen, falls Modul das verlangt
  String cmd = "AT+DMOSETGROUP=0,"; // 12.5 kHz
  cmd += mhz; cmd += ","; cmd += mhz; cmd += ",0000,";
  cmd += String(sq);
  cmd += ",0000";
  draSend(cmd);
}

void draSetVolume(int vol /*0..7 UI -> 1..8 Modul*/) {
  int v = vol + 1; if (v < 1) v = 1; if (v > 8) v = 8;
  draSend("AT+DMOSETVOLUME=" + String(v));
}

void draInit() {
  pinMode(PIN_DRA_PD, OUTPUT);  digitalWrite(PIN_DRA_PD, HIGH);  // aktiv
  pinMode(PIN_DRA_PTT, OUTPUT); digitalWrite(PIN_DRA_PTT, HIGH); // RX
  DRA.begin(9600);
  delay(120);
  draSend("AT+DMOCONNECT");      // Handshake
  delay(80);
  draSend("AT+SETFILTER=0,0,0"); // Pre/De-Emph, HPF, LPF aus
}

// ------------------- Anzeige -------------------
void drawScreen(const char* fstr, int vol, int sql) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  // Zeile 1: Kanal + Frequenz
  display.setCursor(0, 0);
  display.print("CH"); display.print(idxFreq+1); display.print(": ");
  display.print(fstr); display.print(" MHz");

  // Zeile 2: VOL / SQL
  display.setCursor(0, 16);
  display.print("VOL "); display.print(vol); display.print("/7  ");
  display.print("SQL "); display.print(sql); display.print("/9");

  display.display();
}

void showBootLogo() {
  display.clearDisplay();
  // logo_128x32 kommt aus logo_128x32.h (MSB-first)
  display.drawBitmap(0, 0, logo_128x32, 128, 32, SSD1306_WHITE);
  display.display();
  delay(3000);              // 3 s anzeigen
  display.clearDisplay();
  display.display();
}

// ------------------- Setup / Loop -------------------
void setup() {
  Wire.setSDA(PIN_I2C_SDA);
  Wire.setSCL(PIN_I2C_SCL);
  Wire.begin();

  pinMode(PIN_SW_PLUS,  INPUT_PULLUP);
  pinMode(PIN_SW_MINUS, INPUT_PULLUP);

  #if defined(analogReadResolution)
    analogReadResolution(12);   // RP2040 kann 12 Bit
  #endif

  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) { for(;;); }
  showBootLogo();

  // Startwerte aus Potis
  volStep = adcToSteps(analogRead(PIN_POT_VOL), 8);   // 0..7
  sqlStep = adcToSteps(analogRead(PIN_POT_SQL), 10);  // 0..9

  // DRA818 initialisieren & setzen
  draInit();
  draSetGroup(FREQS[idxFreq], sqlStep);
  draSetVolume(volStep);

  drawScreen(FREQS[idxFreq], volStep, sqlStep);
}

void loop() {
  // ---- Potis -> Stufen ----
  static int volLast = -1, sqlLast = -1;

  int volNow = adcToSteps(analogRead(PIN_POT_VOL), 8);   // 0..7
  int sqlNow = adcToSteps(analogRead(PIN_POT_SQL), 10);  // 0..9

  if (volNow != volStep) volStep = volNow;
  if (sqlNow != sqlStep) sqlStep = sqlNow;

  if (volStep != volLast) { draSetVolume(volStep); volLast = volStep; }
  if (sqlStep != sqlLast) { draSetGroup(FREQS[idxFreq], sqlStep); sqlLast = sqlStep; }

  // ---- Drehschalter (+/-) mit Debounce ----
  bool plusNow  = (digitalRead(PIN_SW_PLUS)  == LOW);
  bool minusNow = (digitalRead(PIN_SW_MINUS) == LOW);
  unsigned long now = millis();

  if (plusNow && !lastPlus && (now - lastStepMs > 150)) {
    idxFreq = (idxFreq + 1) % FREQS_N;
    draSetGroup(FREQS[idxFreq], sqlStep);
    lastStepMs = now;
  }
  if (minusNow && !lastMinus && (now - lastStepMs > 150)) {
    idxFreq = (idxFreq - 1 + FREQS_N) % FREQS_N;
    draSetGroup(FREQS[idxFreq], sqlStep);
    lastStepMs = now;
  }
  lastPlus  = plusNow;
  lastMinus = minusNow;

  drawScreen(FREQS[idxFreq], volStep, sqlStep);
  delay(25);
}
