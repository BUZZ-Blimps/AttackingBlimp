#include <Arduino.h>
#include <string.h>
#include <ESP32Servo.h>

using namespace std;

Servo testMotor;

void parseMessage(String newMessage);

void setup() {
  Serial.begin(115200);
  Serial.println("Hello!");
  Serial.setTimeout(0.5);

  testMotor.attach(27); //Left = 26, Right = 27
  testMotor.write(1500);
  Serial.println("Attached to servo: " + String(testMotor.attached()));
}

String message;

void loop() {
  while(Serial.available() > 0){
    char readByte = Serial.read();
    int byteCode = int(readByte);
    if(readByte == '\n'){
      parseMessage(message);

      Serial.print("Received ");
      Serial.println(message);
      message = "";
    }else if(byteCode == 13){
      //Do nothing
    }else{
      message += readByte;
    }
  }
}

void parseMessage(String newMessage){
  bool isDigits = true;
  for(int i=0; i<newMessage.length(); i++) isDigits = isDigits && isDigit(newMessage.charAt(i));

  if(!isDigits){
    Serial.println("Not valid input!");
    return;
  }else{
    int amount = newMessage.toInt();
    if(amount < 0 || amount > 100){
      Serial.println("Input outside range [0,100]!");
      return;
    }

    int thrust = 1500 - 5 * amount;
    testMotor.write(thrust);
    Serial.println("Thrust: " + String(thrust));
  }
}