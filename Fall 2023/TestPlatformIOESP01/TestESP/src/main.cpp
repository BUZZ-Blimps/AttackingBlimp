#include <Arduino.h>

int ledPin = 1;

void setup() {
  pinMode(ledPin, OUTPUT);
  Serial.begin(115200);
}

void loop() {
  Serial.print("HIGH#");
  delay(200);
  digitalWrite(ledPin, HIGH);
  delay(500);
  Serial.print("LOW#");
  delay(200);
  digitalWrite(ledPin, LOW);
  delay(500);
}