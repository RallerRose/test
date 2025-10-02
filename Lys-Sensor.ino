#include <Arduino.h>

const int ldrPin = 32;   // <-- flyt LDR til en ADC1-pin: 32/33/34/35/36/39
const int led1  = 0;
const int led2  = 2;

int threshold = 1300;    // justér efter dine målinger (0–4095)

void setup() {
  Serial.begin(115200);
  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);
  // (valgfrit) lav et par første målinger til at se niveau
  delay(200);
}

void loop() {
  int raw = analogRead(ldrPin);   // 0–4095 på ESP32
  Serial.println(raw);

  // Mørke = høj modstand = høj spænding = høj ADC-værdi (med wiring som vist)
  if (raw > threshold) {          // mørkt → tænd
    digitalWrite(led1, LOW);
    digitalWrite(led2, LOW);
  } else {                        // lyst → sluk
    digitalWrite(led1, HIGH);
    digitalWrite(led2, HIGH);
  }

  delay(10000);
}