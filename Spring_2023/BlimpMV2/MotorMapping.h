#pragma once
#include <ESP32Servo.h>

class MotorMapping{
  public:
  MotorMapping(int yawPin, int pitchPin, int motorPin,double newDeadband, double newTurnOnCom, double newMinCom, double newMaxCom);
  void update(double pitch, double roll, double yaw, double up, double thrust);

  private:
  double motorCom(double command);
  
  Servo yawServo;
  Servo pitchServo;
  Servo motor;
  
   double deadband;
   double turnOnCom;
   double minCom;
   double maxCom;
};
