#include <WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>

/************* Pins *************/
const int trigPinFront = 30;
const int echoPinFront = 28;

const int trigPinLeft = 34;
const int echoPinLeft = 32;

const int trigPinRight = 26;
const int echoPinRight = 24;

const int ENA = 9;  
const int IN1 = 3;
const int IN2 = 2;
const int ENB = 10; 
const int IN3 = 5;
const int IN4 = 4;

/************* Parametre *************/
const int targetDistance = 20;     // ønsket afstand til væggen
const int tolerance = 5;           // tilladt variation
const int frontStopDistance = 15;  // stopafstand for forhindringer


/************* Motor kontrol *************/
int speedLeft = 120;
int speedRight = 120;

void setSpeed(int right, int left) {
  analogWrite(ENA, right);
  analogWrite(ENB, left);
}

void forward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void backward() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void turnRight() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void turnLeft() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void stopAll() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

/************* Sensor funktion *************/
long readDistanceCm(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 25000); // timeout 25ms
  long distance = duration * 0.034 / 2;
  return distance == 0 ? 999 : distance; // hvis intet måles
}

const char* ssid = "DIT_WIFI";
const char* password = "DIT_PASSWORD";

WiFiUDP udp;
const char* host = "192.168.1.100"; // IP på din PC
const int udpPort = 4210;

/************* Setup *************/
unsigned long lastSenseMs = 0;
const unsigned long senseIntervalMs = 100; // 0,1 sek
const int stopDistanceCm = 20;

int currentSensor = 0; // 0 = front, 1 = left, 2 = right
int distFront = 999, distLeft = 999, distRight = 999;

void setup() {
  Serial.begin(115200);
   WiFi.begin(ssid, password);

   while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("WiFi forbundet");

  pinMode(trigPinFront, OUTPUT);
  pinMode(echoPinFront, INPUT);
  pinMode(trigPinLeft, OUTPUT);
  pinMode(echoPinLeft, INPUT);
  pinMode(trigPinRight, OUTPUT);
  pinMode(echoPinRight, INPUT);

  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
}

/************* Hovedlogik *************/
void loop() {
  // Standard: kør frem
  setSpeed(speedRight, speedLeft);
  

  // Skift sensor hver 100ms
  if (millis() - lastSenseMs >= senseIntervalMs) {
    lastSenseMs = millis();

    // Læs alle tre sensorer (i stedet for kun én ad gangen)
    int cmC = readDistanceCm(trigPinFront, echoPinFront);
    int cmL = readDistanceCm(trigPinLeft, echoPinLeft);
    int cmR = readDistanceCm(trigPinRight, echoPinRight);

    // Find nærmeste forhindring
    int nearest = min(cmC, min(cmL, cmR));

    Serial.print("Front: "); Serial.print(cmC);
    Serial.print(" | Venstre: "); Serial.print(cmL);
    Serial.print(" | Højre: "); Serial.print(cmR);
    Serial.print(" | Nærmeste: "); Serial.println(nearest);

    // Reaktionslogik
    if (nearest < stopDistanceCm) {
      stopAll();
      delay(200);
      backward();
      delay(500);

      /*if (cmL > cmR) {
        stopAll();
        delay(500);
        turnRight(); 
      } else {
        stopAll();
        delay(500);
        turnLeft();
      }
      delay(450);
      stopAll();*/
    }
    
    // 2) Ellers følg væg i højre side
    if (cmR < targetDistance - tolerance) {
    // For tæt på væg -> drej lidt venstre
    turnLeft();
    delay(100);   // lille korrektion
    } 
    else if (cmR > targetDistance + tolerance) {
    // For langt fra væg -> drej lidt højre
    turnRight();
    delay(100);
    } 
    else {
    // Indenfor tolerance -> kør lige frem
    forward();
    }
  }
  // Byg JSON
  StaticJsonDocument<200> doc;
  doc["afstand"] = afstand;
  doc["x"] = posX;
  doc["y"] = posY;

  char buffer[200];
  size_t n = serializeJson(doc, buffer);

  // Send til Python
  udp.beginPacket(host, udpPort);
  udp.write((uint8_t*)buffer, n);
  udp.endPacket();

  delay(500);
}