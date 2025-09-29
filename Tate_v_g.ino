/* ===== ESP32 + MPU6050 + 5×Ultralyd + Motorstyring (ingen ledc) -> UDP CSV =====
   Ny CSV (12 felter):
   t_ms,cmF_L,cmF_C,cmF_R,cmSide_F,cmSide_R,gz_dps,ax,ay,az,dEncL,dEncR
   -------------------------------------------------------------------------------
   • IMU: MPU6050 (SDA=13, SCL=32)
   • Ultralyd: 3×front (venstre-vinklet, center, højre-vinklet) + 2×side (for/bag)
   • Motorer: DINE funktionskald m/analogWrite & digitalWrite (ingen ledc direkte)
   • UDP: sender løbende til DEST_IP:DEST_PORT
   ---------------------------------------------------------------------------- */

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "MPU6050.h"

/************* WiFi/UDP *************/
const char* WIFI_SSID = "HOTSPOT-1";
const char* WIFI_PASS = "MercanSpot2024!";
IPAddress    DEST_IP(192,168,0,3);   // ← sæt til din PC’s IP
const uint16_t DEST_PORT = 5005;
WiFiUDP udp;

/************* Ultralyd pins *************/
// 3 forrest (som du havde):
const int trigF_C = 18;  // center trig
const int echoF_C = 34;  // center echo

const int trigF_L = 17;  // venstre-vinklet trig
const int echoF_L = 16;  // venstre-vinklet echo

const int trigF_R = 4;   // højre-vinklet trig
const int echoF_R = 15;   // højre-vinklet echo

// 2 side-sensorer (forrest/bagest på siden, mod væggen):
const int trigSide_F = 19;  // PLACEHOLDER
const int echoSide_F = 21;  // PLACEHOLDER (må gerne være input-only 34..39, men 22 er OK)
const int trigSide_R = 22;   // PLACEHOLDER
const int echoSide_R = 23;  // PLACEHOLDER (input-only pin er OK til echo)

/************* Motor (dine) *************/
const int ENA = 12;  
const int IN1 = 14;
const int IN2 = 27;
const int ENB = 33; 
const int IN3 = 26;
const int IN4 = 25;

/************* Fart/logik *************/
int speedLeft  = 180;   // 0..255
int speedRight = 180;

const int targetDistance     = 20; // cm (bruges ifm. vægfølgning på højre side)
const int tolerance          = 5;
const int stopDistanceCm     = 20; // nødbremse

/************* IMU *************/
const int SDA_PIN = 13;
const int SCL_PIN = 32;
MPU6050 mpu;
static const float GYRO_LSB_PER_DPS = 131.0f;   // ±250 dps
static const float ACC_LSB_PER_G    = 16384.0f; // ±2 g
float gyroZ_bias = 0.0f;

/************* Timing *************/
const uint32_t SAMPLE_SEND_MS = 20;  // ~50 Hz UDP
const uint32_t SENSE_MS       = 100; // ultralyd læsning
unsigned long t0=0, lastSend=0, lastSense=0;

/************* Målebuffer *************/
volatile int cmF_L=999, cmF_C=999, cmF_R=999, cmSide_F=999, cmSide_R=999;

/************* Motor kontrol (dine) *************/
void setSpeed(int right, int left) {
  analogWrite(ENA, right);
  analogWrite(ENB, left);
}

void forward() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void backward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void turnRight() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void turnLeft() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void stopAll() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, HIGH);
}

/************* Ultralyd *************/
long readDistanceCm(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long us = pulseIn(echoPin, HIGH, 25000UL); // 25 ms timeout
  if (us == 0) return 999;
  return (long)(us * 0.034f / 2.0f);
}

/************* Utils *************/
void ensureWifi() {
  if (WiFi.status() == WL_CONNECTED) return;
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  unsigned long t = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t < 15000UL) delay(200);
}

void calibrateGyroZ() {
  const int N=200;
  long sum=0;
  int16_t ax,ay,az,gx,gy,gz;
  for(int i=0;i<10;i++){ mpu.getMotion6(&ax,&ay,&az,&gx,&gy,&gz); delay(5); }
  for(int i=0;i<N;i++){  mpu.getMotion6(&ax,&ay,&az,&gx,&gy,&gz); sum+=gz; delay(5); }
  gyroZ_bias = (sum/(float)N) / GYRO_LSB_PER_DPS;
}

/************* Setup *************/
void setup() {
  Serial.begin(115200);

  // Pins
  pinMode(trigF_C, OUTPUT); pinMode(echoF_C, INPUT);
  pinMode(trigF_L, OUTPUT); pinMode(echoF_L, INPUT);
  pinMode(trigF_R, OUTPUT); pinMode(echoF_R, INPUT);

  pinMode(trigSide_F, OUTPUT); pinMode(echoSide_F, INPUT);
  pinMode(trigSide_R, OUTPUT); pinMode(echoSide_R, INPUT);

  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  setSpeed(0,0); stopAll();

  // I2C/IMU
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  mpu.initialize(); // 0x68
  if(!mpu.testConnection()){
    Serial.println("MPU6050 FAIL — tjek SDA=13, SCL=32, 3.3V, GND, AD0.");
    while(1) delay(1000);
  }
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  Serial.println("# Kalibrerer gyroZ… hold i ro"); calibrateGyroZ();

  // WiFi
  ensureWifi();
  if (WiFi.status()==WL_CONNECTED) { Serial.print("# IP="); Serial.println(WiFi.localIP()); }

  t0 = millis(); lastSend = lastSense = 0;

  Serial.println("# Header: t_ms,cmF_L,cmF_C,cmF_R,cmSide_F,cmSide_R,gz_dps,ax,ay,az,dEncL,dEncR");
}

/************* Loop *************/
void loop() {
  if (WiFi.status()!=WL_CONNECTED) ensureWifi();

  // Fast cruise + simpel vægfølgning (brug side-sensorer)
  setSpeed(speedRight, speedLeft);

  if (millis()-lastSense >= SENSE_MS) {
    lastSense = millis();

    // Læs alle 5
    int fL = readDistanceCm(trigF_L,  echoF_L);
    int fC = readDistanceCm(trigF_C,  echoF_C);
    int fR = readDistanceCm(trigF_R,  echoF_R);
    int sF = readDistanceCm(trigSide_F, echoSide_F);
    int sR = readDistanceCm(trigSide_R, echoSide_R);

    cmF_L=fL; cmF_C=fC; cmF_R=fR; cmSide_F=sF; cmSide_R=sR;

    // Nødbremse
    int nearest = min(fC, min(fL, min(fR, min(sF, sR))));
    if (nearest < stopDistanceCm) {
      stopAll(); setSpeed(0,0);
      delay(120);
      backward(); setSpeed(speedRight, speedLeft); delay(220);
      forward();
    }

    // Vægfølgning (højre side): brug side for/bag som “vinkel-fejl”
    // positiv diff = forreste side-sensor længere fra væg end bageste -> peg lidt mod væg (drej højre)
    int sideErr = sF - sR;       // retningstegn kan byttes hvis dine sensorer sidder modsat
    int distErr = sF - targetDistance;

    if (abs(sideErr) > 5) {
      if (sideErr > 0) {  // peg ud fra væg -> drej højre
        turnRight(); delay(70); forward();
      } else {            // peg ind mod væg -> drej venstre
        turnLeft();  delay(70); forward();
      }
    } else if (abs(distErr) > tolerance) {
      if (distErr > 0) {  // for langt fra væg -> drej højre lidt
        turnRight(); delay(60); forward();
      } else {            // for tæt på væg -> drej venstre lidt
        turnLeft();  delay(60); forward();
      }
    } else {
      forward();
    }
  }

  // UDP-send (50 Hz)
  unsigned long now = millis();
  if (now - lastSend >= SAMPLE_SEND_MS) {
    lastSend = now;
    unsigned long t_ms = now - t0;

    // IMU
    int16_t axr, ayr, azr, gxr, gyr, gzr;
    mpu.getMotion6(&axr,&ayr,&azr,&gxr,&gyr,&gzr);
    float ax = axr / ACC_LSB_PER_G;
    float ay = ayr / ACC_LSB_PER_G;
    float az = azr / ACC_LSB_PER_G;
    float gz_dps = (gzr / GYRO_LSB_PER_DPS) - gyroZ_bias;

    char line[220];
    int n = snprintf(line, sizeof(line),
      "%lu,%d,%d,%d,%d,%d,%.2f,%.3f,%.3f,%.3f,%d,%d",
      (unsigned long)t_ms,cmF_L,cmF_C,cmF_R,cmSide_F,cmSide_R,gz_dps,ax,ay,az,0,0

    );

    if (WiFi.status()==WL_CONNECTED && n>0) {
      udp.beginPacket(DEST_IP, DEST_PORT);
      udp.write((const uint8_t*)line, (size_t)n);
      udp.endPacket();
    }

    Serial.println(line);
  }
  
}
