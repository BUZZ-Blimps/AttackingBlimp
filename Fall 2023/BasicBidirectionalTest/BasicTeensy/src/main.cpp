#include <Arduino.h>
#include "SerialHandler.h"

SerialHandler espHandler;

void setup(){

}
float lastSent = 0; 

void loop(){
  espHandler.ParseMessages();

  float currentTime = millis()/1000.0;
  if(currentTime - lastSent > 1.0){
    lastSent = currentTime;
    Serial.print("Hi! (");
    Serial.print(currentTime);
    Serial.println(")");
  }
}