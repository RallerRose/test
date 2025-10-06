#include <Arduino.h>
#include <Wire.h>

/************* Pins *************/
// Front (uændret)
const int trigPinFront = 18;
const int echoPinFront = 34;   // input-only

// Venstre (90° mod væg)  ——> FLYT echo væk fra 21
const int trigPinF90 = 19;
const int echoPinF90 = 35;     

// Højre (90°)  ——> FLYT trig væk fra 22
const int trigPinR90 = 5;      
const int echoPinR90 = 23;  

// (valgfrie)
const int trigPinLeft  = 17;
const int echoPinLeft  = 16;
const int trigPinRight = 4;    
const int echoPinRight = 15;


// Motorer (L298N / H-bridge)
const int ENA = 12;  // Højre PWM
const int IN1 = 14;  // Højre retning
const int IN2 = 27;  // (strap-pin på nogle ESP32 — hold lav ved boot)
const int ENB = 33;  // Venstre PWM
const int IN3 = 26;  // Venstre retning
const int IN4 = 25;

// Encodere (LM393)
//const int encLeftPin  = 32;  // venstre hjul
//const int encRightPin = 33;  // højre hjul

/************* Parametre *************/
const int TARGET_SIDE_CM   = 20; // ønsket afstand til venstre væg
const int FRONT_TURN_CM    = 15; // under dette → 90° højresving
const int FRONT_SLOW_CM    = 30; // under dette → skaler basisfart
const int MAX_PWM          = 255;

// Små faste trims (gør højre lidt langsommere)
const int LEFT_TRIM  = 0;
const int RIGHT_TRIM = 8;   // 5–12 typisk

// Grundhastigheder (kan justeres løbende)
int baseLeft  = 140;
int baseRight = 145;

// Sidekorrektion (PD)
float Kp = 4.5f;          // proportional bid (cm → PWM)
float Kd = 2.0f;          // dæmper zig-zag
const int CORR_MAX = 80;  // max styrekorrektion i PWM
const int CTRL_DT_MS = 20;
const int ERR_DEADBAND_CM = 1; // lille dødbånd

// For at undgå bak i normal vægfølgning:
const int MIN_FWD_PWM = 70;

/************* Geometri til 90°-sving *************/
const float wheelDiameter_mm = 65.0f;
const float ticksPerRev      = 40.0f;        // sæt korrekt for jeres encoder
const float TRACK_MM         = 120.0f;       // mål hjulafstand (midt–midt)
const float mmPerTick        = (PI * wheelDiameter_mm) / ticksPerRev;
// Teori: arc for 90° pivot = pi*TRACK/4
const int   TICKS_TURN_90    = (int)round((TRACK_MM / (4.0f * wheelDiameter_mm)) * ticksPerRev);

/************* Encoder / hastighed *************/
volatile uint32_t ticksL = 0, ticksR = 0;
volatile uint32_t lastTickL_us = 0, lastTickR_us = 0;
const uint32_t minTickSpacingUs = 300;

unsigned long lastSpeedMs = 0;
uint32_t prevTL = 0, prevTR = 0;
float vL_f = 0, vR_f = 0;
const float VEL_ALPHA = 0.5f;

// PI-match (gør venstre/højre lige hurtige)
// +bias: højre er hurtigere → træk fra højre, læg til venstre
float Kv = 3.0f;   // P-gain (cm/s → PWM)
float Ki = 0.8f;   // I-gain
float bias = 0.0f;
const float BIAS_MAX = 25.0f;
const int   BIAS_SLEW = 5;

// HMC5883L (0x1E) – enkel init/læs/heading
#define MAG_SDA   21
#define MAG_SCL   22
#define MAG_ADDR  0x1E

// Justér hvis modulet vender “skævt”
#define MAG_SWAP_XY  0      // 0 normal, 1 byt X/Y
#define MAG_FLIP_X   1      // 1 normal, -1 invertér X
#define MAG_FLIP_Y   1      // 1 normal, -1 invertér Y
const float MAG_MOUNT_OFFSET_DEG = -30.0f;   // din værdi

static inline void mapAxes(int16_t x,int16_t y, float &mx,float &my){
  float rx = x;
  float ry = y;
  mx = (MAG_SWAP_XY ? ry : rx) * (float)MAG_FLIP_X;
  my = (MAG_SWAP_XY ? rx : ry) * (float)MAG_FLIP_Y;
}


// --- Robust I2C: langsom clock, timeout, retries og bus-clear ved fejl ---
static void i2cClearBus(int sclPin=MAG_SCL, int sdaPin=MAG_SDA) {
  pinMode(sclPin, OUTPUT_OPEN_DRAIN);
  pinMode(sdaPin, INPUT_PULLUP);
  for (int i=0; i<9 && digitalRead(sdaPin)==LOW; ++i) {
    digitalWrite(sclPin, HIGH); delayMicroseconds(5'000);
    digitalWrite(sclPin, LOW);  delayMicroseconds(5'000);
  }
  digitalWrite(sclPin, HIGH); delayMicroseconds(5'000);
  pinMode(sdaPin, OUTPUT_OPEN_DRAIN);
  digitalWrite(sdaPin, HIGH); delayMicroseconds(5'000);
}

static bool magInit(){
  Wire.begin(MAG_SDA, MAG_SCL);
  Wire.setClock(50000);     // 50 kHz = mere tolerant end 100 kHz
  Wire.setTimeOut(30);      // ms – så hænger requestFrom ikke
  // CRA: 8x avg, 15 Hz, normal
  Wire.beginTransmission(MAG_ADDR); Wire.write(0x00); Wire.write(0x70); if (Wire.endTransmission()!=0) return false;
  // CRB: gain 1.3 Ga
  Wire.beginTransmission(MAG_ADDR); Wire.write(0x01); Wire.write(0x20); if (Wire.endTransmission()!=0) return false;
  // Mode: continuous
  Wire.beginTransmission(MAG_ADDR); Wire.write(0x02); Wire.write(0x00); if (Wire.endTransmission()!=0) return false;
  delay(10);
  return true;
}

static bool magReadXYZ_once(int16_t &x,int16_t &y,int16_t &z){
  Wire.beginTransmission(MAG_ADDR); Wire.write(0x03);
  if (Wire.endTransmission(false)!=0) return false;
  if (Wire.requestFrom(MAG_ADDR,(uint8_t)6,true)!=6) return false;
  uint8_t b[6]; for(int i=0;i<6;i++) b[i]=Wire.read();
  x=(int16_t)((b[0]<<8)|b[1]);  // X
  z=(int16_t)((b[2]<<8)|b[3]);  // Z
  y=(int16_t)((b[4]<<8)|b[5]);  // Y
  if (x==-4096 || y==-4096 || z==-4096) return false; // overflow
  return true;
}

static bool magReadXYZ(int16_t &x,int16_t &y,int16_t &z){
  // op til 3 forsøg; hvis 2. fejler -> bus-clear + reinit
  for (int attempt=0; attempt<3; ++attempt){
    if (magReadXYZ_once(x,y,z)) return true;
    if (attempt==0) { delay(5); continue; }
    if (attempt==1) { i2cClearBus(); magInit(); delay(10); continue; }
  }
  return false;
}


// Returnér kompasretning i grader [0..360)
static float magHeadingDeg(){
  int16_t x,y,z;
  if (!magReadXYZ(x,y,z)) return NAN;   // ingen re-init her
  float mx,my;
  mapAxes(x,y,mx,my);                   // <--- manglede!
  const float decl = 3.5f; // ~DK
  float hd = atan2f(my, mx) * 180.0f / PI + decl - MAG_MOUNT_OFFSET_DEG;
  if (hd < 0)   hd += 360.0f;
  if (hd >= 360.0f) hd -= 360.0f;
  return hd;
}



// Hjælpere til vinkler
static inline float angDiffDeg(float a, float b){
  float d = a - b;
  while (d > 180.f) d -= 360.f;
  while (d < -180.f) d += 360.f;
  return d; // i [-180..180]
}

/************* Utils *************/
int clampi(int v, int lo, int hi){ return v<lo?lo:(v>hi?hi:v); }
int slew(int cur, int tgt, int step){ if (tgt>cur) return min(tgt,cur+step); if (tgt<cur) return max(tgt,cur-step); return cur; }

/************* Encoders ISR *************/
void IRAM_ATTR isrEncLeft()  {
  uint32_t n = micros();
  if (n - lastTickL_us >= minTickSpacingUs) { ticksL++; lastTickL_us = n; }
}
void IRAM_ATTR isrEncRight() {
  uint32_t n = micros();
  if (n - lastTickR_us >= minTickSpacingUs) { ticksR++; lastTickR_us = n; }
}

/************* Ultralyd *************/
int readUltrasonicCM(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(8);
  digitalWrite(trigPin, LOW);
  unsigned long us = pulseIn(echoPin, HIGH, 30000UL);
  if (us == 0) return 400;       // intet ekko → “meget langt væk”
  return (int)((us * 0.5f) / 29.412f + 0.5f);
}

/************* Motorstyring (uden LEDC) *************/
void setRightMotor(int pwm, bool forward) {
  pwm = clampi(pwm, 0, MAX_PWM);
  digitalWrite(IN1, forward ? LOW : HIGH);  // Højre frem: IN1 LOW, IN2 HIGH
  digitalWrite(IN2, forward ? HIGH : LOW);
  analogWrite(ENA, pwm);
}
void setLeftMotor(int pwm, bool forward) {
  pwm = clampi(pwm, 0, MAX_PWM);
  digitalWrite(IN3, forward ? HIGH : LOW);  // Venstre frem: IN3 HIGH, IN4 LOW
  digitalWrite(IN4, forward ? LOW  : HIGH);
  analogWrite(ENB, pwm);
}
void stopAll() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  // valgfri bremse:
  // digitalWrite(IN1, HIGH); digitalWrite(IN2, HIGH);
  // digitalWrite(IN3, HIGH); digitalWrite(IN4, HIGH);
}

/************* Køre-funktioner *************/
void forwardDrive(int pwmL, int pwmR) {
  // Gulv så vi ikke ender i utilsigtet bak ved normal vægfølgning
  pwmL = clampi(pwmL, MIN_FWD_PWM, MAX_PWM);
  pwmR = clampi(pwmR, MIN_FWD_PWM, MAX_PWM);
  setLeftMotor (pwmL, true);
  setRightMotor(pwmR, true);
}
void spinRight(int pwm = 170) {  // venstre frem, højre tilbage
  setLeftMotor (pwm, true);
  setRightMotor(pwm, false);
}

/************* 90° højresving m. encodere *************/
void turnRight90_mag() {
  if (!magInit()) { Serial.println("[MAG] init fail, fallback encoders"); return /*eller kald din encoder-funktion*/; }

  // Læs start-retning og nulstil akkumuleret vinkel
  float hPrev = magHeadingDeg();
  while (isnan(hPrev)) { delay(10); hPrev = magHeadingDeg(); }
  float accum = 0.0f;

  // Start pivot: venstre frem, højre tilbage
  const int pwmTurn = 150;
  setLeftMotor (pwmTurn, true);
  setRightMotor(pwmTurn, false);

  const uint32_t T_MAX = 3000;     // sikkerhedstimeout (ms)
  const float    TARGET = -90.0f;  // højresving = negativ akkumuleret vinkel
  const float    TOL    = 4.0f;    // ± tolerance i grader
  const uint32_t MIN_SPIN_MS = 200;

  uint32_t t0 = millis();
  uint32_t last = t0;

  while (millis() - t0 < T_MAX) {
    float h = magHeadingDeg();
    if (!isnan(h)) {
      float d = angDiffDeg(h, hPrev);        // signed step [-180..180]
      hPrev = h;
      // Smid urealistiske spring (spike filter)
      if (fabsf(d) < 40.0f) accum += d;      // 40°/sample er rigeligt ved ~8–20 Hz

      // Stop-kriterie: vi har roteret ~ -90° og kørt lidt tid
      if ((millis() - t0) > MIN_SPIN_MS && fabsf(accum - TARGET) <= TOL) break;
    }
    delay(15);
  }

  stopAll(); delay(60);
  // “fald i hak” en anelse frem
  forwardDrive(clampi(baseLeft, 80, 230), clampi(baseRight, 80, 230));
  delay(120);
  stopAll();
  Serial.printf("[MAG] turn accum=%.1f°\n", accum);
}


/************* Hastigheds-PI (opdater ~10 Hz) *************/
void updateSpeedPI() {
  unsigned long now = millis();
  if (now - lastSpeedMs < 100) return;
  uint32_t dMs = now - lastSpeedMs; lastSpeedMs = now;

  noInterrupts(); uint32_t tL = ticksL, tR = ticksR; interrupts();
  uint32_t dTL = tL - prevTL, dTR = tR - prevTR; prevTL = tL; prevTR = tR;

  float vL = (dMs > 0) ? ((dTL * mmPerTick) / 10.0f) / (dMs / 1000.0f) : 0.0f;
  float vR = (dMs > 0) ? ((dTR * mmPerTick) / 10.0f) / (dMs / 1000.0f) : 0.0f;
  vL_f = (1.0f - VEL_ALPHA) * vL_f + VEL_ALPHA * vL;
  vR_f = (1.0f - VEL_ALPHA) * vR_f + VEL_ALPHA * vR;

  // eV > 0 ⇒ højre hurtigere end venstre
  float eV = vR_f - vL_f;

  static float I = 0.0f;
  I += eV * (dMs / 1000.0f);
  float maxI = (BIAS_MAX) / max(0.001f, Ki);
  I = constrain(I, -maxI, maxI);

  float biasTarget = Kv * eV + Ki * I;
  biasTarget = constrain(biasTarget, -BIAS_MAX, BIAS_MAX);

  int cur = (int)round(bias);
  int tgt = (int)round(biasTarget);
  bias = (float)slew(cur, tgt, BIAS_SLEW);

  // Debug (valgfri)
  // Serial.printf("vR=%.1f vL=%.1f eV=%.2f bias=%.1f\n", vR_f, vL_f, eV, bias);
}

// --- PD-styring state ---
float prevErr = 0.0f;
unsigned long lastCtrlMs = 0;

/************* Setup *************/
void setup() {
  Serial.begin(115200);

  magInit();   // klar magnetometeret fra start

  // Sensorpins
  pinMode(trigPinFront, OUTPUT);
  pinMode(echoPinFront, INPUT);
  pinMode(trigPinF90, OUTPUT);
  pinMode(echoPinF90, INPUT);
  pinMode(trigPinR90,OUTPUT);
  pinMode(echoPinR90,INPUT);

  // (valgfri log)
  pinMode(trigPinLeft,  OUTPUT); pinMode(echoPinLeft,  INPUT);
  pinMode(trigPinRight, OUTPUT); pinMode(echoPinRight, INPUT);

  // Motorretning
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  // Sørg for sikre niveauer ved boot (vigtigt for GPIO12)
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  stopAll();
  delay(150);

  // Encodere
  //pinMode(encLeftPin,  INPUT_PULLUP);
  //pinMode(encRightPin, INPUT_PULLUP);
  //attachInterrupt(digitalPinToInterrupt(encLeftPin),  isrEncLeft,  RISING);
  //attachInterrupt(digitalPinToInterrupt(encRightPin), isrEncRight, RISING);

  lastSpeedMs = millis();
  lastCtrlMs  = millis();

  Serial.println("Start OK (no LEDC)");
}

/************* Loop *************/
void loop() {
  // 1) Læs sensorer
  int f = readUltrasonicCM(trigPinFront,  echoPinFront);
  int s = readUltrasonicCM(trigPinF90, echoPinF90); // venstre væg (90°)

  // 2) 90° højresving når fronten er meget tæt
  if (f > 0 && f < FRONT_TURN_CM) {
    Serial.println("[TURN] 90 deg right");
    stopAll(); delay(40);
    turnRight90_mag();
    return;
  }
  
  float h = magHeadingDeg();
  if (!isnan(h)) {
    Serial.printf("Heading = %.1f°\n", h);
  } else {
    Serial.println("Mag read fail");
  }
  delay(200);  // ca. 5 Hz

  // 3) Opdater hastigheds-PI (holder L/R lige)
  updateSpeedPI();

  // 4) Fartnedskalering hvis noget er foran (uden at bakke)
  int baseDynL = baseLeft;
  int baseDynR = baseRight;
  if (f > 0 && f < FRONT_SLOW_CM) {
    // skaler begge baser lineært mellem minCruise og maxBase når f går fra FRONT_TURN_CM→FRONT_SLOW_CM
    const int lo = FRONT_TURN_CM, hi = FRONT_SLOW_CM;
    const int minCruise = 100;
    int cl = clampi(f, lo, hi);
    int scaled = minCruise + (int)((long)(cl - lo) * (max(baseLeft, baseRight) - minCruise) / (hi - lo));
    baseDynL = clampi(scaled, 80, 230);
    baseDynR = clampi(scaled, 80, 230);
  }

  // 5) Venstre-væg PD-kontrol
  unsigned long now = millis();
  if (now - lastCtrlMs >= CTRL_DT_MS) {
    float dt = (now - lastCtrlMs) / 1000.0f;
    lastCtrlMs = now;

    // Fejl: positiv når vi er for TÆT på væggen (så skal vi styre mod højre)
    // e = TARGET - målt
    float e = (float)TARGET_SIDE_CM - (float)s;

    // D-led
    float de = (e - prevErr) / max(0.001f, dt);
    prevErr = e;

    // Dødbånd for at undgå flakken
    if (fabs(e) < ERR_DEADBAND_CM) e = 0.0f;

    // Hvis ingen ekko (s≈400), så “søger” vi let mod væggen (dvs. drej lidt venstre)
    if (s >= 380) {
      e = -2.0f;   // lille negativ fejl → u negativ → drej venstre
      de = 0.0f;
    }

    // Styre-korrektion u (PWM)
    float uf = Kp * e + Kd * de;
    int   u  = clampi((int)lroundf(uf), -CORR_MAX, CORR_MAX);

    // Sammensæt PWM’er
    // u>0 ⇒ for tæt på venstre væg ⇒ drej HØJRE: højre langsommere, venstre hurtigere
    int pwmL = baseDynL + LEFT_TRIM + (int)lroundf(bias) + u;
    int pwmR = baseDynR - RIGHT_TRIM - (int)lroundf(bias) - u;

    // Gulv/loft og kør
    pwmL = clampi(pwmL, MIN_FWD_PWM, MAX_PWM);
    pwmR = clampi(pwmR, MIN_FWD_PWM, MAX_PWM);
    forwardDrive(pwmL, pwmR);

    // Debug (valgfri)
     //Serial.printf("f=%d s=%d e=%.2f u=%d bias=%.1f pwmL=%d pwmR=%d\n", f, s, e, u, bias, pwmL, pwmR);
     //Serial.print(hPrev);
  }

  // lille pause så loop ikke spinner alt for hurtigt mellem kontrolopdateringer
  delay(2);
}
