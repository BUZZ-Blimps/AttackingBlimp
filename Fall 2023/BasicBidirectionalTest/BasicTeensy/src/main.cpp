#include <Arduino.h>
#include "ROSHandler.h"
#include "NonBlockingTimer.h"

ROSHandler rosHandler;

NonBlockingTimer timer_pub;

void callback(String message){
  Serial.print("Received topic message: \"");
  Serial.print(message);
  Serial.println("\".");
}

void setup(){
  rosHandler.Init();
  rosHandler.SubscribeTopic_String("testTopic", callback);
  timer_pub.setFrequency(5);
}

float lastSent = 0;
void loop(){
  rosHandler.Update();

  if(millis() - lastSent > 1000){
    lastSent = millis();
    Serial.println("Hi!");
  }

  if(timer_pub.isReady()){
    rosHandler.PublishTopic_Float64("TeensyTopic", millis());
  }
}