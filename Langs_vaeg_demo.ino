// Demo_WallFollow_ESP32.ino
// Stabil venstre-vægfølgning + 90° højresving + frontnedbremsning
// Pins og struktur matcher vores seneste opsætning (Rapport final).

#include <Arduino.h>
#include <Wire.h>

/* ---------- PIN-MAP (ESP32) ---------- */
// Front ultralyd (midt)
const int TRIG_FRONT = 18;
const int ECHO_FRONT = 34;  // ADC1 input-only (ok til echo)

// Venstre side (90° mod væg)
const int TRIG_L90   = 19;
const int ECHO_L90   = 35;  // ADC1 input-only (ok til echo)

// Højre 90° (front skrå/hjørnedetektion / demo)
const int TRIG_R90   = 5;
const int ECHO_R90   = 23;

// (Valgfri, hvis du har dem monteret)
const int TRIG_LEFT  = 17;
const int ECHO_LEFT  = 16;
const int TRIG_RIGHT = 4;
const int ECHO_RIGHT = 15;

// L298N motorer
const int ENA = 12;  // Højre PWM  (GPIO12: hold niveau lav ved boot)
const int IN1 = 14;  // Højre retning
const int IN2 = 27;  // (strap-pin; vi holder den lav ved boot)
const int ENB = 33;  // Venstre PWM
const int IN3 = 26;  // Venstre retning
const int IN4 = 25;

const int ldrPin = 32;   // <-- flyt LDR til en ADC1-pin: 32/33/34/35/36/39
const int led1  = 0;
const int led2  = 2;

int threshold = 1300;    // justér efter dine målinger (0–4095)

// I2C kompas (HMC5883L/GY-271)
#define MAG_SDA 22
#define MAG_SCL 21
#define MAG_ADDR 0x1E

/* ---------- TUNES ---------- */
const int   TARGET_SIDE_CM = 20; // ønsket afstand til venstre væg
const int   FRONT_TURN_CM  = 15; // under dette: 90° højresving
const int   FRONT_SLOW_CM  = 30; // under dette: lineær nedskalering af fart
const int   MAX_PWM        = 255;
const int   MIN_FWD_PWM    = 70;

int BASE_L = 140;  // grundfart venstre
int BASE_R = 145;  // grundfart højre

// Små trims (udligner mekanisk forskel)
const int LEFT_TRIM  = 0;
const int RIGHT_TRIM = 8;

// PD-regulator for sideafstand
float Kp = 4.5f;
float Kd = 2.0f;
const int   CORR_MAX        = 80;
const int   CTRL_DT_MS      = 20;
const int   ERR_DEADBAND_CM = 1;

// Magnetometer mounting/offets (justér hvis modulet vender skævt)
#define MAG_SWAP_XY  0
#define MAG_FLIP_X   1
#define MAG_FLIP_Y   1
const float MAG_MOUNT_OFFSET_DEG = -30.0f;
const float MAG_DECL_DEG         = 3.5f; // DK ca. +/-

/* ---------- UTIL ---------- */
static inline int clampi(int v, int lo, int hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

static inline int slew(int cur, int tgt, int step) {
  if (tgt > cur) return min(tgt, cur + step);
  if (tgt < cur) return max(tgt, cur - step);
  return cur;
}

/* ---------- ULTRALYD ---------- */
static int readUltrasonicCM(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(8);
  digitalWrite(trigPin, LOW);
  unsigned long us = pulseIn(echoPin, HIGH, 30000UL);
  if (us == 0) return 400; // intet ekko => “langt væk”
  return (int)((us * 0.5f) / 29.412f + 0.5f);
}

/* ---------- MOTORER (uden LEDC for enkelhed) ---------- */
static void setRightMotor(int pwm, bool forward) {
  pwm = clampi(pwm, 0, MAX_PWM);
  digitalWrite(IN1, forward ? LOW : HIGH);
  digitalWrite(IN2, forward ? HIGH : LOW);
  analogWrite(ENA, pwm);
}
static void setLeftMotor(int pwm, bool forward) {
  pwm = clampi(pwm, 0, MAX_PWM);
  digitalWrite(IN3, forward ? HIGH : LOW);
  digitalWrite(IN4, forward ? LOW  : HIGH);
  analogWrite(ENB, pwm);
}
static void stopAll() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

/* ---------- MAGNETOMETER (robust I2C) ---------- */
static void i2cClearBus(int sclPin = MAG_SCL, int sdaPin = MAG_SDA) {
  pinMode(sclPin, OUTPUT_OPEN_DRAIN);
  pinMode(sdaPin, INPUT_PULLUP);
  for (int i = 0; i < 9 && digitalRead(sdaPin) == LOW; ++i) {
    digitalWrite(sclPin, HIGH); delayMicroseconds(5000);
    digitalWrite(sclPin, LOW);  delayMicroseconds(5000);
  }
  digitalWrite(sclPin, HIGH); delayMicroseconds(5000);
  pinMode(sdaPin, OUTPUT_OPEN_DRAIN);
  digitalWrite(sdaPin, HIGH); delayMicroseconds(5000);
}

static bool magInit() {
  Wire.begin(MAG_SDA, MAG_SCL);
  Wire.setClock(50000); // langsomt = stabilt
  Wire.setTimeOut(30);
  // CRA: 8x avg, 15 Hz
  Wire.beginTransmission(MAG_ADDR); Wire.write(0x00); Wire.write(0x70);
  if (Wire.endTransmission() != 0) return false;
  // CRB: gain
  Wire.beginTransmission(MAG_ADDR); Wire.write(0x01); Wire.write(0x20);
  if (Wire.endTransmission() != 0) return false;
  // Mode: continuous
  Wire.beginTransmission(MAG_ADDR); Wire.write(0x02); Wire.write(0x00);
  if (Wire.endTransmission() != 0) return false;
  delay(10);
  return true;
}

static bool magReadXYZ_once(int16_t &x, int16_t &y, int16_t &z) {
  Wire.beginTransmission(MAG_ADDR); Wire.write(0x03);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MAG_ADDR, (uint8_t)6, true) != 6) return false;
  uint8_t b[6]; for (int i = 0; i < 6; ++i) b[i] = Wire.read();
  x = (int16_t)((b[0] << 8) | b[1]); // X
  z = (int16_t)((b[2] << 8) | b[3]); // Z
  y = (int16_t)((b[4] << 8) | b[5]); // Y
  if (x == -4096 || y == -4096 || z == -4096) return false;
  return true;
}

static bool magReadXYZ(int16_t &x, int16_t &y, int16_t &z) {
  for (int attempt = 0; attempt < 3; ++attempt) {
    if (magReadXYZ_once(x, y, z)) return true;
    if (attempt == 0) { delay(5); continue; }
    if (attempt == 1) { i2cClearBus(); magInit(); delay(10); continue; }
  }
  return false;
}

static inline void mapAxes(int16_t x, int16_t y, float &mx, float &my) {
  float rx = x, ry = y;
  mx = (MAG_SWAP_XY ? ry : rx) * (float)MAG_FLIP_X;
  my = (MAG_SWAP_XY ? rx : ry) * (float)MAG_FLIP_Y;
}

static float magHeadingDeg() {
  int16_t x, y, z;
  if (!magReadXYZ(x, y, z)) return NAN;
  float mx, my; mapAxes(x, y, mx, my);
  float hd = atan2f(my, mx) * 180.0f / PI + MAG_DECL_DEG - MAG_MOUNT_OFFSET_DEG;
  if (hd < 0)   hd += 360.0f;
  if (hd >= 360.0f) hd -= 360.0f;
  return hd;
}

static inline float angDiffDeg(float a, float b) {
  float d = a - b;
  while (d > 180.f) d -= 360.f;
  while (d < -180.f) d += 360.f;
  return d;
}

/* ---------- Sving: 90° højre (magnetometer, timed fallback) ---------- */
static void turnRight90() {
  bool magOK = magInit();
  if (!magOK) { Serial.println("[TURN] MAG init fail -> timed fallback"); }

  // Start pivot: venstre frem, højre tilbage
  const int pwmTurn = 150;
  setLeftMotor(pwmTurn, true);
  setRightMotor(pwmTurn, false);

  uint32_t t0 = millis();

  if (magOK) {
    float hPrev = magHeadingDeg();
    while (isnan(hPrev)) { delay(10); hPrev = magHeadingDeg(); }
    float accum = 0.0f;
    const uint32_t T_MAX = 3000;
    const float TARGET = -90.0f; // højre
    const float TOL = 4.0f;

    while (millis() - t0 < T_MAX) {
      float h = magHeadingDeg();
      if (!isnan(h)) {
        float d = angDiffDeg(h, hPrev);
        hPrev = h;
        if (fabsf(d) < 40.0f) accum += d;
        if ((millis() - t0) > 220 && fabsf(accum - TARGET) <= TOL) break;
      }
      delay(15);
    }
  } else {
    // Timet fallback (~650–850 ms afh. gulv/friktion)
    delay(750);
  }

  // Stop + lille “hak” frem for at falde i den nye retning
  stopAll(); delay(60);
  int pwmL = clampi(BASE_L, 80, 230);
  int pwmR = clampi(BASE_R, 80, 230);
  setLeftMotor(pwmL, true);
  setRightMotor(pwmR, true);
  delay(120);
  stopAll();
}

/* ---------- PD-state ---------- */
float prevErr = 0.0f;
unsigned long lastCtrlMs = 0;

/* ---------- SETUP ---------- */
void setup() {
  Serial.begin(115200);

  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);

  // Sensorpins
  pinMode(TRIG_FRONT, OUTPUT); pinMode(ECHO_FRONT, INPUT);
  pinMode(TRIG_L90,   OUTPUT); pinMode(ECHO_L90,   INPUT);
  pinMode(TRIG_R90,   OUTPUT); pinMode(ECHO_R90,   INPUT);

  pinMode(TRIG_LEFT,  OUTPUT); pinMode(ECHO_LEFT,  INPUT);
  pinMode(TRIG_RIGHT, OUTPUT); pinMode(ECHO_RIGHT, INPUT);

  // Motorretning + sikre niveauer ved boot
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  stopAll();
  delay(150);

  // I2C (mag)
  magInit(); // best effort

  lastCtrlMs = millis();

  Serial.println("Wall-follow demo ready.");
}

/* ---------- LOOP ---------- */
void loop() {
  // 1) Sekventiel og kortfattet sensorlæsning (minimer kryds-ekko)
  int cmF  = readUltrasonicCM(TRIG_FRONT, ECHO_FRONT); delay(5);
  int cmL  = readUltrasonicCM(TRIG_L90,   ECHO_L90);   delay(5);
  int cmR  = readUltrasonicCM(TRIG_R90,   ECHO_R90);   // (hjørnehjælp)
  int raw = analogRead(ldrPin); 

  // 2) Hårdt hjørne? -> 90° højresving
  if (cmF > 0 && cmF < FRONT_TURN_CM) {
    stopAll(); delay(40);
    turnRight90();
    return; // begynd ny runde efter sving
  }

  // 3) Dynamisk frontnedbremsning (uden bak)
  int baseL = BASE_L;
  int baseR = BASE_R;
  if (cmF > 0 && cmF < FRONT_SLOW_CM) {
    const int lo = FRONT_TURN_CM, hi = FRONT_SLOW_CM;
    const int minCruise = 100;
    int c = clampi(cmF, lo, hi);
    int scaled = minCruise + (int)((long)(c - lo) * (max(BASE_L, BASE_R) - minCruise) / (hi - lo));
    baseL = clampi(scaled, 80, 230);
    baseR = clampi(scaled, 80, 230);
  }

  // 4) Venstre-væg PD-regulering
  unsigned long now = millis();
  if (now - lastCtrlMs >= CTRL_DT_MS) {
    float dt = (now - lastCtrlMs) / 1000.0f;
    lastCtrlMs = now;

    // Fejl: positiv hvis vi er for tæt på venstre væg (skal styre mod højre)
    float e = (float)TARGET_SIDE_CM - (float)cmL;

    // Dødbånd
    if (fabs(e) < ERR_DEADBAND_CM) e = 0.0f;

    // Hvis venstre sensor "mister" væggen -> søg let mod venstre
    float de = (e - prevErr) / max(0.001f, dt);
    if (cmL >= 380) { e = -2.0f; de = 0.0f; }
    prevErr = e;

    // PD-output
    int u = clampi((int)lroundf(Kp * e + Kd * de), -CORR_MAX, CORR_MAX);

    // PWM-sammensætning (u>0 => for tæt -> drej højre => højre langsommere)
    int pwmL = baseL + LEFT_TRIM + u;
    int pwmR = baseR - RIGHT_TRIM - u;

    // Gulv/loft og kør frem
    pwmL = clampi(pwmL, MIN_FWD_PWM, MAX_PWM);
    pwmR = clampi(pwmR, MIN_FWD_PWM, MAX_PWM);
    setLeftMotor (pwmL, true);
    setRightMotor(pwmR, true);

    // (Valgfrit) Debug kort:
    // Serial.printf("F=%d L=%d R=%d e=%.1f u=%d pwmL=%d pwmR=%d\n", cmF, cmL, cmR, e, u, pwmL, pwmR);
  }

  if (raw > threshold) {          // mørkt → tænd
      digitalWrite(led1, LOW);
      digitalWrite(led2, LOW);
  } else {                        // lyst → sluk
      digitalWrite(led1, HIGH);
      digitalWrite(led2, HIGH);
  }

  // 5) Lidt ro, så vi ikke banker ekkoerne sammen
  delay(2);
}
