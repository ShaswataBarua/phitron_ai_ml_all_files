/*
  BUBT Robot Line Race - Arduino Nano (RoboCampusBD board, TB6612FNG)
  16-sensor mux array + SSD1306 OLED + 4 buttons + dual motor driver

  Rules:
   - Line 3 cm. Sharp turns, crossover, gaps, inverse line, round/square.
   - Finish = black square (25 cm). Robot stops on it and the race ends.
   - Line lost -> robot searches by itself and finds the line again.
   - Score: time bonus = (limit - lap time). Round 1 = 300 s, Round 2 = 600 s.

  CONVENTION: sensor 0 = robot's LEFT edge, sensor 15 = RIGHT edge.
  Black line gives LOW analog reading (white = HIGH) in normal mode.
  Inverse mode = white line on black background.
*/

#include <Wire.h>
#include <EEPROM.h>
#include <ezButton.h>
#include "SSD1306Ascii.h"
#include "SSD1306AsciiWire.h"

// ---------------- Pins ----------------
// TB6612FNG: channel A (AO1/AO2) = RIGHT motor, channel B (BO1/BO2) = LEFT motor
#define AIN1 12
#define AIN2 11
#define PWMA 10
#define BIN1 7
#define BIN2 8
#define PWMB 9

#define BTN_UP     A1
#define BTN_DOWN   A2
#define BTN_SELECT A3
#define BTN_BACK   2

#define S0 3               // Mux select (PD3..PD6 for fast port write)
#define S1 4
#define S2 5
#define S3 6
#define MUX_SIG A0

// ---------------- Build options ----------------
#define SENSOR_ORDER_REVERSED 0   // 1 if sensor 15 is on the robot's LEFT
#define FAST_PWM 1                // 31 kHz PWM on pins 9/10

// ---------------- Tuning constants ----------------
const int   MUX_SETTLE_US   = 4;
const int   edge            = 800;
const int   NOISE_FLOOR     = 12;
const long  LINE_MIN_TOTAL  = 30;
const float KpBoost         = 0.65f;
const float SpeedDrop       = 0.08f;
const int   MinSpeed        = 40;
const int   CalibSpeed      = 80;
const int   PivotSpeed      = 120;
const int   SHARP_POS       = 300;
const uint16_t SEARCH_FLIP_MS  = 900;
const uint16_t LOST_TIMEOUT_MS = 15000;  // line 15 s e na pele stop (auto recovery er por)
const uint16_t SWEEP_MS        = 600;
const uint16_t RAMP_MS         = 400;
const int      BLOB_ON         = 30;
const long     BLOB_MIN_MASS   = 60;
const uint8_t  WIDE_BLOB       = 6;
const float    DEADBAND        = 25;
const float    STRAIGHT_BAND   = 40;
const uint16_t STRAIGHT_MS     = 250;
const int      STRAIGHT_BOOST  = 60;

// ---------------- Finish = black square ----------------
const uint8_t FINISH_ON  = 10;   // Act >= 10 = black square
const uint8_t FINISH_OFF = 6;    // Act <= 6 = square theke baire
const uint16_t FINISH_MS    = 150;  // square e ei somoy dhore thakle thamben
const int      SQUARE_CRAWL = 110;  // square e steering na kore ei speed e straight
const int weight[16] = {
  -edge, -520, -360, -200,
  -100,   -50,  -25,  -10,
   10,     25,   50,  100,
   200,   360,  520, edge
};

// ---------------- Saved settings ----------------
struct Settings {
  int MIN[16];
  int MAX[16];
  float Kp, Ki, Kd;
  int BaseSpeed, MaxSpeed;
  uint16_t gapMs;
  uint8_t raceRound;
  uint8_t revFlags;   // bit0 leftRev, bit1 rightRev, bit2 !preferRight, bit3 inverseLine
  uint16_t key;
};
const uint16_t SETTINGS_KEY = 8812;   // ager key theke alada, tai purono data load hobe na -> recalibrate koro
const int EEPROM_ADDRESS = 0;

float Kp = 0.85f, Ki = 0.00f, Kd = 0.35f;
int BaseSpeed = 160, MaxSpeed = 210;
uint16_t GapMs = 450;
uint8_t raceRound = 1;               // 1 = 300 s bonus limit, 2 = 600 s
bool calibrated = false;
bool preferRight = true;
bool leftRev = true, rightRev = true;
bool inverseLine = false;            // false = black line on white, true = white line on black

// ---------------- Sensor data ----------------
int arrayReading[16];
int minValue[16];
int maxValue[16];
int normalized[16];
long gTotal = 0;
uint8_t gActive = 0;
float gPos = 0;

// ---------------- Objects ----------------
ezButton btnUp(BTN_UP);
ezButton btnDown(BTN_DOWN);
ezButton btnSelect(BTN_SELECT);
ezButton btnBack(BTN_BACK);
#define I2C_ADDR 0x3C
SSD1306AsciiWire oled;

// ---------------- Menu ----------------
enum MenuState { ST_MAIN, ST_RUN, ST_CAL_AUTO, ST_CAL_HAND, ST_PID, ST_SPEED, ST_SENSORS, ST_MOTOR };
MenuState currentState = ST_MAIN;
bool needRedraw = true;
int mainIdx = 0, pidIdx = 0, speedIdx = 0, motorIdx = 0;
const int MAIN_COUNT = 7;
const char* mainItems[MAIN_COUNT] = {
  "1.Start Run", "2.Auto Calib", "3.Hand Calib",
  "4.PID Tuning", "5.Speed/Gap", "6.Live Sensors", "7.Motor Test"
};
unsigned long lastSensorUpdate = 0;

enum RunResult { RUN_FINISHED, RUN_ABORTED, RUN_LOST };
unsigned long raceStartMs = 0;
unsigned long raceEndMs = 0;

// =====================================================
//                      SETUP
// =====================================================
void setup() {
  Wire.begin();
  Wire.setClock(400000L);
  oled.begin(&Adafruit128x64, I2C_ADDR);
  oled.setFont(System5x7);
  oled.clear();

  btnUp.setDebounceTime(35);
  btnDown.setDebounceTime(35);
  btnSelect.setDebounceTime(35);
  btnBack.setDebounceTime(35);

  pinMode(S0, OUTPUT); pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT); pinMode(S3, OUTPUT);
  pinMode(MUX_SIG, INPUT);

  pinMode(PWMA, OUTPUT); pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT); pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);

  ADCSRA = (ADCSRA & 0xF8) | 0x05;
#if FAST_PWM
  TCCR1B = (TCCR1B & 0xF8) | 0x01;
#endif

  stopMotors();
  loadSettings();
}

void loop() {
  btnUp.loop(); btnDown.loop(); btnSelect.loop(); btnBack.loop();

  switch (currentState) {
    case ST_MAIN:     handleMainMenu(); break;
    case ST_RUN:      handleRun(); break;
    case ST_CAL_AUTO: handleCalibrate(true); break;
    case ST_CAL_HAND: handleCalibrate(false); break;
    case ST_PID:      handlePidMenu(); break;
    case ST_SPEED:    handleSpeedMenu(); break;
    case ST_SENSORS:  handleSensorData(); break;
    case ST_MOTOR:    handleMotorTest(); break;
  }
}

// =====================================================
//                   SENSOR ENGINE
// =====================================================
void readSensors() {
  for (uint8_t i = 0; i < 16; i++) {
    PORTD = (PORTD & ~0x78) | (i << 3);
    delayMicroseconds(MUX_SETTLE_US);
    uint8_t idx = SENSOR_ORDER_REVERSED ? (15 - i) : i;
    arrayReading[idx] = analogRead(MUX_SIG);
  }
}

// Read + normalize + compute position, total, active count
void readAndProcess() {
  readSensors();
  long sum = 0, total = 0;
  uint8_t active = 0;
  for (uint8_t i = 0; i < 16; i++) {
    long diff = (long)maxValue[i] - minValue[i];
    int n = 0;
    if (diff > 0) {
      long t = (long)(arrayReading[i] - minValue[i]) * 100L / diff;
      t = constrain(t, 0, 100);
      // Normal: black = high n.  Inverse: white = high n.
      n = inverseLine ? (int)t : (100 - (int)t);
      if (n < NOISE_FLOOR) n = 0;
    }
    normalized[i] = n;
    sum += (long)weight[i] * n;
    total += n;
    if (n > 50) active++;
  }
  gTotal = total;
  gActive = active;
  gPos = (total > 0) ? (float)sum / (float)total : 0;
}

// Loops / forks: follow only the RIGHT-most (or LEFT-most) branch
void applySplitBias() {
  int8_t st[8], en[8];
  uint8_t n = 0;
  int i = 0;
  while (i < 16 && n < 8) {
    if (normalized[i] > BLOB_ON) {
      int a = i;
      long m = 0;
      while (i < 16 && normalized[i] > BLOB_ON) { m += normalized[i]; i++; }
      if (m >= BLOB_MIN_MASS) { st[n] = a; en[n] = i - 1; n++; }
    } else i++;
  }

  // One wide patch touching only one edge = side branch -> take it
  if (n == 1) {
    int w = en[0] - st[0] + 1;
    if (w >= WIDE_BLOB) {
      bool tr = en[0] >= 13, tl = st[0] <= 2;
      int keepA = -1, keepB = -1;
      if (preferRight && tr && !tl)  { keepA = en[0] - 3; keepB = en[0]; }
      if (!preferRight && tl && !tr) { keepA = st[0];     keepB = st[0] + 3; }
      if (keepA >= 0) {
        long sum1 = 0, tot1 = 0;
        for (int k = 0; k < 16; k++) {
          if (k < keepA || k > keepB) normalized[k] = 0;
          sum1 += (long)weight[k] * normalized[k];
          tot1 += normalized[k];
        }
        gTotal = tot1;
        gPos = (tot1 > 0) ? (float)sum1 / (float)tot1 : 0;
      }
    }
    return;
  }
  if (n < 2) return;

  uint8_t pick = preferRight ? (n - 1) : 0;
  long sum = 0, total = 0;
  for (int k = 0; k < 16; k++) {
    if (k < st[pick] - 1 || k > en[pick] + 1) normalized[k] = 0;
    sum += (long)weight[k] * normalized[k];
    total += normalized[k];
  }
  gTotal = total;
  gPos = (total > 0) ? (float)sum / (float)total : 0;
}

void printClock(unsigned long t) {
  oled.print(t / 60000); oled.print(':');
  unsigned long s = (t / 1000) % 60;
  if (s < 10) oled.print('0');
  oled.print(s);
}

void drawRunTime() {
  oled.set2X();
  oled.setCursor(0, 2);
  printClock(millis() - raceStartMs);
  oled.print(F("  "));
  oled.set1X();
}

// =====================================================
//                    RACE ENGINE
// =====================================================
RunResult runRace() {
  oled.clear();
  oled.setCursor(0, 0); oled.print(F("RUNNING"));
  oled.setCursor(0, 6); oled.print(F("BACK = stop"));
  drawRunTime();

  unsigned long runStart = millis();
  unsigned long lastDraw = runStart;
  unsigned long lostSince = 0, straightSince = 0;
  unsigned long lastUs = micros();
  float prev = 0, integ = 0, lastPos = 0, dFilt = 0;
  int lastSide = 1;

    bool armed = false;            // white line e ekbar namle true
  unsigned long squareSince = 0; // square e prothom kokhon dhorechhilo
  unsigned long squareLast  = 0; // square shesh kokhon dekhechhilo

  while (true) {
    btnBack.loop();
    if (btnBack.isPressed()) { stopMotors(); raceEndMs = millis(); return RUN_ABORTED; }

    readAndProcess();
    unsigned long now = millis();
    if (now - lastDraw >= 1000) { lastDraw = now; drawRunTime(); }

        // ---------- Black square = finish (time-based) ----------
    if (gActive <= FINISH_OFF) armed = true;          // white line e ekbar namle ready

    if (armed && gActive >= FINISH_ON) {
      if (!squareSince) squareSince = now;
      squareLast = now;
    } else if (squareSince && (now - squareLast > 40)) {
      squareSince = 0;                                // 40 ms er beshi square chharha = reset
    }

    if (squareSince) {
      if (now - squareSince >= FINISH_MS) {
        brakeMotors();
        raceEndMs = now;
        return RUN_FINISHED;
      }
      drive(SQUARE_CRAWL, SQUARE_CRAWL);              // square e steering na, straight
      continue;
    }

    applySplitBias();

    // ---------- Line present: PID ----------
    if (gTotal >= LINE_MIN_TOTAL) {
      float pos = gPos;
      if (lostSince) { lostSince = 0; prev = pos; integ = 0; dFilt = 0; }
      lastPos = pos;
      lastSide = (pos >= 0) ? 1 : -1;

      float absE = fabs(pos);
      float pe = (absE < DEADBAND) ? 0.0f : (pos > 0 ? pos - DEADBAND : pos + DEADBAND);
      float kp = Kp + KpBoost * (absE / (float)edge);
      float kd = Kd * (1.0f + 0.5f * (absE / (float)edge));
      integ = constrain(integ + pe, -800, 800);

      unsigned long us = micros();
      float dtUs = (float)(us - lastUs);
      lastUs = us;
      if (dtUs < 200.0f) dtUs = 200.0f;
      float rawD = (pos - prev) * (2000.0f / dtUs);
      dFilt += 0.25f * (rawD - dFilt);
      prev = pos;

      float pid = pe * kp + integ * Ki + dFilt * kd;

      if (absE < STRAIGHT_BAND) { if (!straightSince) straightSince = now; }
      else straightSince = 0;
      bool fast = straightSince && (now - straightSince > STRAIGHT_MS);
      if (fast) pid *= 0.45f;

      float spd = fast ? (float)(BaseSpeed + STRAIGHT_BOOST) : (BaseSpeed - absE * SpeedDrop);
      spd = constrain(spd, MinSpeed, MaxSpeed);
      if (absE > 0) {} // placeholder kept minimal
      unsigned long since = now - runStart;
      if (since < RAMP_MS) spd *= (float)since / (float)RAMP_MS;

      float l = constrain(spd + pid, -MaxSpeed, MaxSpeed);
      float r = constrain(spd - pid, -MaxSpeed, MaxSpeed);
      drive((int)l, (int)r);
    }
    // ---------- Line lost: auto recovery ----------
    else {
      if (!lostSince) lostSince = now;
      unsigned long lost = now - lostSince;
      bool sharp = fabs(lastPos) > SHARP_POS;

      if (!sharp && lost < GapMs) {
        int gs = max(MinSpeed, (int)(BaseSpeed * 0.85f));
        drive(gs, gs);                                   // gap er upor diye straight
      } else {
        if (lost > LOST_TIMEOUT_MS) { stopMotors(); raceEndMs = now; return RUN_LOST; }
        unsigned long t = sharp ? lost : (lost - GapMs);
        int dir = (t < SEARCH_FLIP_MS) ? lastSide : -lastSide;
        drive(dir * PivotSpeed, -dir * PivotSpeed);      // line khunje: spin
      }
    }
  }
}

void handleRun() {
  if (!calibrated) {
    msg(F(" CALIBRATE FIRST!"), 1200);
    goMenu();
    return;
  }

  oled.clear();
  oled.setCursor(0, 1); oled.print(F("READY - place on START"));
  oled.setCursor(0, 3); oled.print(F("SELECT = GO"));
  oled.setCursor(0, 5); oled.print(F("BACK   = menu"));
  if (!waitSelectOrBack()) { goMenu(); return; }
  delay(800);

  raceStartMs = millis();
  RunResult r = runRace();

  unsigned long t = raceEndMs - raceStartMs;
  oled.clear();
  oled.setCursor(0, 0);
  if (r == RUN_FINISHED)   oled.print(F("FINISHED!"));
  else if (r == RUN_LOST)  oled.print(F("LINE NOT FOUND"));
  else                     oled.print(F("STOPPED"));

  oled.set2X();
  oled.setCursor(0, 2);
  oled.print(t / 1000); oled.print('.');
  unsigned long cs = (t % 1000) / 10;
  if (cs < 10) oled.print('0');
  oled.print(cs); oled.print('s');
  oled.set1X();

  if (r == RUN_FINISHED) {
    unsigned long limitS = (raceRound == 1) ? 300UL : 600UL;
    unsigned long used = t / 1000;
    unsigned long bonus = (used < limitS) ? (limitS - used) : 0;
    oled.setCursor(0, 5); oled.print(F("Time bonus: ")); oled.print(bonus);
  }
  oled.setCursor(0, 7); oled.print(F("Hold SELECT 2s = menu"));
  waitLongSelect(2000);
  goMenu();
}

// =====================================================
//                    CALIBRATION
// =====================================================
void resetMinMax() {
  for (uint8_t i = 0; i < 16; i++) { minValue[i] = 1023; maxValue[i] = 0; }
}

void sampleMinMax() {
  readSensors();
  for (uint8_t j = 0; j < 16; j++) {
    if (arrayReading[j] < minValue[j]) minValue[j] = arrayReading[j];
    if (arrayReading[j] > maxValue[j]) maxValue[j] = arrayReading[j];
  }
}

void sweepFor(int l, int r, unsigned long ms) {
  drive(l, r);
  unsigned long t0 = millis();
  while (millis() - t0 < ms) sampleMinMax();
}

void centerOnLine() {
  unsigned long t0 = millis();
  int dir = 1;
  while (millis() - t0 < 2000) {
    readAndProcess();
    if (gTotal >= LINE_MIN_TOTAL) {
      if (fabs(gPos) < 40) break;
      dir = (gPos > 0) ? 1 : -1;
    }
    drive(dir * (CalibSpeed - 20), -dir * (CalibSpeed - 20));
  }
  stopMotors();
}

void handleCalibrate(bool autoMode) {
  resetMinMax();
  oled.clear();

  if (autoMode) {
    oled.setCursor(0, 2); oled.print(F("  CALIBRATING..."));
    oled.setCursor(0, 4); oled.print(F(" Robot will swing"));
    sweepFor(-CalibSpeed,  CalibSpeed, SWEEP_MS);
    sweepFor( CalibSpeed, -CalibSpeed, SWEEP_MS * 2);
    sweepFor(-CalibSpeed,  CalibSpeed, SWEEP_MS);
    stopMotors();
    calibrated = true;
    centerOnLine();
  } else {
    oled.setCursor(0, 0); oled.print(F("HAND CALIBRATION"));
    oled.setCursor(0, 2); oled.print(F("Slide robot over"));
    oled.setCursor(0, 3); oled.print(F("black AND white"));
    unsigned long t0 = millis();
    int lastSec = -1;
    while (millis() - t0 < 6000) {
      sampleMinMax();
      int sec = 6 - (int)((millis() - t0) / 1000);
      if (sec != lastSec) {
        lastSec = sec;
        oled.setCursor(0, 6); oled.print(F("Time left: ")); oled.print(sec); oled.print(' ');
      }
    }
    calibrated = true;
  }

  saveSettings();

  uint8_t bad = 0;
  for (uint8_t i = 0; i < 16; i++) if ((maxValue[i] - minValue[i]) < 150) bad++;

  oled.clear();
  oled.setCursor(0, 2);
  if (bad == 0) oled.print(F(" Calibration OK!"));
  else { oled.print(F(" WEAK sensors: ")); oled.print(bad); }
  delay(1500);
  goMenu();
}

// =====================================================
//                    MENU SYSTEM
// =====================================================
void handleMainMenu() {
  if (btnUp.isPressed())   { mainIdx = (mainIdx - 1 + MAIN_COUNT) % MAIN_COUNT; needRedraw = true; }
  if (btnDown.isPressed()) { mainIdx = (mainIdx + 1) % MAIN_COUNT; needRedraw = true; }
  if (btnSelect.isPressed()) {
    switch (mainIdx) {
      case 0: currentState = ST_RUN; break;
      case 1: currentState = ST_CAL_AUTO; break;
      case 2: currentState = ST_CAL_HAND; break;
      case 3: currentState = ST_PID; pidIdx = 0; break;
      case 4: currentState = ST_SPEED; speedIdx = 0; break;
      case 5: currentState = ST_SENSORS; break;
      case 6: currentState = ST_MOTOR; motorIdx = 0; break;
    }
    needRedraw = true;
    return;
  }
  if (needRedraw) {
    oled.clear();
    oled.setCursor(0, 0); oled.print(F("  Team Break_Point "));
    for (int i = 0; i < MAIN_COUNT; i++) {
      oled.setCursor(0, i + 1);
      oled.print(i == mainIdx ? "> " : "  ");
      oled.print(mainItems[i]);
    }
    needRedraw = false;
  }
}

float stepf(float v, float d) {
  v += d;
  if (v < 0) v = 0;
  return roundf(v * 100.0f) / 100.0f;
}

void handlePidMenu() {
  if (btnUp.isPressed()) {
    if (pidIdx == 0) Kp = stepf(Kp, 0.05f);
    else if (pidIdx == 1) Ki = stepf(Ki, 0.01f);
    else if (pidIdx == 2) Kd = stepf(Kd, 0.05f);
    needRedraw = true;
  }
  if (btnDown.isPressed()) {
    if (pidIdx == 0) Kp = stepf(Kp, -0.05f);
    else if (pidIdx == 1) Ki = stepf(Ki, -0.01f);
    else if (pidIdx == 2) Kd = stepf(Kd, -0.05f);
    needRedraw = true;
  }
  if (btnSelect.isPressed()) {
    if (pidIdx == 3) { saveSettings(); msg(F("  PID Saved!"), 800); goMenu(); return; }
    pidIdx++;
    needRedraw = true;
  }
  if (btnBack.isPressed()) { goMenu(); return; }

  if (needRedraw) {
    oled.clear();
    oled.setCursor(0, 0); oled.print(F("--- PID TUNING ---"));
    oled.setCursor(0, 2); oled.print(pidIdx == 0 ? "> " : "  "); oled.print(F("Kp: ")); oled.print(Kp, 2);
    oled.setCursor(0, 3); oled.print(pidIdx == 1 ? "> " : "  "); oled.print(F("Ki: ")); oled.print(Ki, 2);
    oled.setCursor(0, 4); oled.print(pidIdx == 2 ? "> " : "  "); oled.print(F("Kd: ")); oled.print(Kd, 2);
    oled.setCursor(0, 6); oled.print(pidIdx == 3 ? "> [ SAVE & EXIT ]" : "  [ SAVE & EXIT ]");
    needRedraw = false;
  }
}

// Speed menu: 0 Base, 1 Max, 2 Gap, 3 Round, 4 Split, 5 Inverse, 6 Save
void handleSpeedMenu() {
  if (btnUp.isPressed() || btnDown.isPressed()) {
    int d = btnUp.isPressed() ? 1 : -1;
    if (speedIdx == 0)      BaseSpeed = constrain(BaseSpeed + 5 * d, 40, 255);
    else if (speedIdx == 1) MaxSpeed  = constrain(MaxSpeed  + 5 * d, 40, 255);
    else if (speedIdx == 2) GapMs     = constrain((int)GapMs + 50 * d, 100, 1500);
    else if (speedIdx == 3) raceRound = (raceRound == 1) ? 2 : 1;
    else if (speedIdx == 4) preferRight = !preferRight;
    else if (speedIdx == 5) inverseLine = !inverseLine;
    needRedraw = true;
  }
  if (btnSelect.isPressed()) {
    if (speedIdx == 6) { saveSettings(); msg(F(" Settings Saved!"), 800); goMenu(); return; }
    speedIdx++;
    needRedraw = true;
  }
  if (btnBack.isPressed()) { goMenu(); return; }

  if (needRedraw) {
    oled.clear();
    oled.setCursor(0, 0); oled.print(F("-- SPEED / GAP --"));
    oled.setCursor(0, 1); oled.print(speedIdx == 0 ? "> " : "  "); oled.print(F("Base  : ")); oled.print(BaseSpeed);
    oled.setCursor(0, 2); oled.print(speedIdx == 1 ? "> " : "  "); oled.print(F("Max   : ")); oled.print(MaxSpeed);
    oled.setCursor(0, 3); oled.print(speedIdx == 2 ? "> " : "  "); oled.print(F("Gap ms: ")); oled.print(GapMs);
    oled.setCursor(0, 4); oled.print(speedIdx == 3 ? "> " : "  "); oled.print(F("Round : ")); oled.print(raceRound);
    oled.setCursor(0, 5); oled.print(speedIdx == 4 ? "> " : "  "); oled.print(F("Split : ")); oled.print(preferRight ? F("RIGHT") : F("LEFT "));
    oled.setCursor(0, 6); oled.print(speedIdx == 5 ? "> " : "  "); oled.print(F("Inverse: ")); oled.print(inverseLine ? F("YES") : F("NO "));
    oled.setCursor(0, 7); oled.print(speedIdx == 6 ? "> [ SAVE & EXIT ]" : "  [ SAVE & EXIT ]");
    needRedraw = false;
  }
}

void handleSensorData() {
  if (btnBack.isPressed()) { goMenu(); return; }

  if (needRedraw) {
    oled.clear();
    oled.setCursor(0, 0); oled.print(F("SENSORS L>R (0-9)"));
    oled.setCursor(0, 2); oled.print(F("0-7 : "));
    oled.setCursor(0, 4); oled.print(F("8-15: "));
    needRedraw = false;
  }

  if (millis() - lastSensorUpdate > 90) {
    lastSensorUpdate = millis();
    readAndProcess();

    oled.setCursor(36, 2);
    for (int i = 0; i < 8; i++)  { oled.print(min(9, normalized[i] / 10)); oled.print(' '); }
    oled.setCursor(36, 4);
    for (int i = 8; i < 16; i++) { oled.print(min(9, normalized[i] / 10)); oled.print(' '); }

    oled.setCursor(0, 6);
    oled.print(F("Pos: ")); oled.print((long)gPos);
    oled.print(F("  Act: ")); oled.print(gActive);
    oled.clearToEOL();

    oled.setCursor(0, 7);
    oled.print(F("Finish: "));
    oled.print(gActive >= FINISH_ON ? F("YES ") : F("no  "));
    oled.print(inverseLine ? F("INV") : F("NRM"));
    oled.clearToEOL();
  }
}

void handleMotorTest() {
  if (btnUp.isPressed()) {
    if (motorIdx == 0) leftRev = !leftRev;
    else if (motorIdx == 1) rightRev = !rightRev;
    needRedraw = true;
  }
  if (btnDown.isPressed() && motorIdx < 2) {
    if (motorIdx == 0) setLeft(120); else setRight(120);
    delay(700);
    stopMotors();
  }
  if (btnSelect.isPressed()) {
    if (motorIdx == 2) { saveSettings(); msg(F(" Motors Saved!"), 800); goMenu(); return; }
    motorIdx++;
    needRedraw = true;
  }
  if (btnBack.isPressed()) { goMenu(); return; }

  if (needRedraw) {
    oled.clear();
    oled.setCursor(0, 0); oled.print(F("--- MOTOR TEST ---"));
    oled.setCursor(0, 2); oled.print(motorIdx == 0 ? "> " : "  "); oled.print(F("Left  flip: ")); oled.print(leftRev ? F("YES") : F("NO "));
    oled.setCursor(0, 3); oled.print(motorIdx == 1 ? "> " : "  "); oled.print(F("Right flip: ")); oled.print(rightRev ? F("YES") : F("NO "));
    oled.setCursor(0, 5); oled.print(F("DOWN=spin  UP=flip"));
    oled.setCursor(0, 7); oled.print(motorIdx == 2 ? "> [ SAVE & EXIT ]" : "  [ SAVE & EXIT ]");
    needRedraw = false;
  }
}

// =====================================================
//                      HELPERS
// =====================================================
void goMenu() { currentState = ST_MAIN; needRedraw = true; }

void msg(const __FlashStringHelper* s, unsigned int ms) {
  oled.clear();
  oled.setCursor(0, 3);
  oled.print(s);
  delay(ms);
}

// Result screen stays until SELECT is HELD for holdMs
void waitLongSelect(uint16_t holdMs) {
  unsigned long t0 = millis();
  while (digitalRead(BTN_SELECT) == LOW && millis() - t0 < 1500) {}
  delay(50);

  unsigned long pressedAt = 0;
  int lastBar = -1;
  while (true) {
    if (digitalRead(BTN_SELECT) == LOW) {
      if (!pressedAt) pressedAt = millis();
      unsigned long held = millis() - pressedAt;
      if (held >= holdMs) break;
      int bar = (int)(held * 16UL / holdMs);
      if (bar != lastBar) {
        lastBar = bar;
        oled.setCursor(0, 7);
        for (int i = 0; i < 16; i++) oled.print(i < bar ? '#' : '-');
        oled.clearToEOL();
      }
    } else if (pressedAt) {
      pressedAt = 0;
      lastBar = -1;
      oled.setCursor(0, 7);
      oled.print(F("Hold SELECT 2s=menu"));
      oled.clearToEOL();
    }
  }
  while (digitalRead(BTN_SELECT) == LOW) {}
  delay(50);
}

bool waitSelectOrBack() {
  while (true) {
    btnSelect.loop(); btnBack.loop();
    if (btnSelect.isPressed()) return true;
    if (btnBack.isPressed())   return false;
  }
}

void loadSettings() {
  Settings d;
  EEPROM.get(EEPROM_ADDRESS, d);
  if (d.key == SETTINGS_KEY) {
    for (uint8_t i = 0; i < 16; i++) { minValue[i] = d.MIN[i]; maxValue[i] = d.MAX[i]; }
    Kp = d.Kp; Ki = d.Ki; Kd = d.Kd;
    BaseSpeed = d.BaseSpeed; MaxSpeed = d.MaxSpeed;
    GapMs = d.gapMs;
    raceRound = (d.raceRound == 2) ? 2 : 1;
    leftRev    = (d.revFlags & 1) != 0;
    rightRev   = (d.revFlags & 2) != 0;
    preferRight = (d.revFlags & 4) == 0;
    inverseLine = (d.revFlags & 8) != 0;
    calibrated = true;
  } else {
    resetMinMax();
    calibrated = false;
  }
}

void saveSettings() {
  Settings d;
  for (uint8_t i = 0; i < 16; i++) { d.MIN[i] = minValue[i]; d.MAX[i] = maxValue[i]; }
  d.Kp = Kp; d.Ki = Ki; d.Kd = Kd;
  d.BaseSpeed = BaseSpeed; d.MaxSpeed = MaxSpeed;
  d.gapMs = GapMs;
  d.raceRound = raceRound;
  d.revFlags = (leftRev ? 1 : 0) | (rightRev ? 2 : 0) | (preferRight ? 0 : 4) | (inverseLine ? 8 : 0);
  d.key = SETTINGS_KEY;
  EEPROM.put(EEPROM_ADDRESS, d);
}

// =====================================================
//                      MOTORS
// =====================================================
void motor(uint8_t pwm, uint8_t in1, uint8_t in2, int v) {
  v = constrain(v, -255, 255);
  if (v >= 0) { digitalWrite(in1, HIGH); digitalWrite(in2, LOW);  analogWrite(pwm, v); }
  else        { digitalWrite(in1, LOW);  digitalWrite(in2, HIGH); analogWrite(pwm, -v); }
}

void setLeft(int v)  { motor(PWMB, BIN1, BIN2, leftRev  ? -v : v); }
void setRight(int v) { motor(PWMA, AIN1, AIN2, rightRev ? -v : v); }

void drive(int l, int r) {
  setLeft(l);
  setRight(r);
}

void stopMotors() {
  analogWrite(PWMA, 0); analogWrite(PWMB, 0);
  digitalWrite(AIN1, LOW); digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW); digitalWrite(BIN2, LOW);
}

void brakeMotors() {
  digitalWrite(AIN1, HIGH); digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, HIGH); digitalWrite(BIN2, HIGH);
  analogWrite(PWMA, 255); analogWrite(PWMB, 255);
  delay(100);
  stopMotors();
}