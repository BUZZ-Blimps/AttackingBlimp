#include <Arduino.h>

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
}

String total = "";
char delimiter = '#';

void loop() {
  while(Serial1.available() > 0){
    char current = Serial1.read();
    if(current != delimiter){
      total += current;
    }else{
      Serial.print("Received: ");
      Serial.println(total);
      total = "";
    }
  }
}