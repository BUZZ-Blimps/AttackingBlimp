#include <Arduino.h>
#include "ROSHandler.h"

ROSHandler rosHandler;

void callback(String message){
  Serial.print("Received topic message: \"");
  Serial.print(message);
  Serial.println("\".");
}

void setup(){
  rosHandler.Init();
  rosHandler.SubscribeTopic_String("testTopic", callback);
}

void loop(){
  rosHandler.Update();
}