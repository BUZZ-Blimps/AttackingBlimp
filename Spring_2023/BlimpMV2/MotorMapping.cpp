#include "MotorMapping.h"

 MotorMapping::MotorMapping(int yawPin, int pitchPin, int motorPin,double newDeadband, double newTurnOnCom, double newMinCom, double newMaxCom){
  deadband = newDeadband;
  turnOnCom = newTurnOnCom;
  minCom = newMinCom;
  maxCom = newMaxCom;
  
  //attach to pin
  yawServo.attach(yawPin);
  pitchServo.attach(pitchPin);
  motor.attach(motorPin);
  
  //initialize
  yawServo.write(90);
  pitchServo.write(135);
  motor.write(1500);
}

void MotorMapping::update(double pitch, double roll,double yaw, double up, double forward) {

  bool debug = false;

  double pi = 3.14159265;

  roll = -roll*pi/180;
  pitch = pitch*pi/180;

  double x = forward*cos(pitch)+sin(pitch)*sin(roll)*yaw+sin(pitch)*cos(roll)*up;
  double y = yaw*cos(roll)-up*sin(roll);
  double z = -forward*sin(pitch)+yaw*cos(pitch)*sin(roll)+up*cos(pitch)*cos(roll);

  forward = x;
  yaw = y;
  up = z;

  if (debug) Serial.println("Corrected Inputs");
  if (debug) Serial.print("Forward: ");
  if (debug) Serial.print(forward);
  if (debug) Serial.print("\tYaw: ");
  if (debug) Serial.print(yaw);
  if (debug) Serial.print("\tUp: ");
  if (debug) Serial.println(up);
  
  double thrust = sqrt(pow(yaw,2)+pow(up,2)+pow(forward,2));
  
  double theta1 = atan2(yaw,forward)*180/pi;
  double phi1 = asin(up/thrust)*180/pi;

  double theta2 = atan2(-yaw,-forward)*180/pi;
  double phi2 = asin(-up/thrust)*180/pi;

  double theta3 = theta1;
  double phi3 = phi1-180;

  double theta4 = theta2;
  double phi4 = phi2-180;
  
  if (debug) Serial.println("Initial Solutions");
  if (debug) Serial.print(theta1);
  if (debug) Serial.print("\t");
  if (debug) Serial.print(theta2);
  if (debug) Serial.print("\t");
  if (debug) Serial.print(theta3);
  if (debug) Serial.print("\t");
  if (debug) Serial.println(theta4);

  if (debug) Serial.print(phi1);
  if (debug) Serial.print("\t");
  if (debug) Serial.print(phi2);
  if (debug) Serial.print("\t");
  if (debug) Serial.print(phi3);
  if (debug) Serial.print("\t");
  if (debug) Serial.println(phi4);
  if (debug) Serial.println();

  double thetaOffset = 135;
  double phiOffset = 90;

  theta1 += thetaOffset;
  theta2 += thetaOffset;
  theta3 += thetaOffset;
  theta4 += thetaOffset;
  phi1 += phiOffset;
  phi2 += phiOffset;
  phi3 += phiOffset;
  phi4 += phiOffset;

  if (debug) Serial.print("Shifted Solutions");
  if (debug) Serial.print(theta1);
  if (debug) Serial.print("\t");
  if (debug) Serial.print(theta2);
  if (debug) Serial.print("\t");
  if (debug) Serial.print(theta3);
  if (debug) Serial.print("\t");
  if (debug) Serial.println(theta4);

  if (debug) Serial.print(phi1);
  if (debug) Serial.print("\t");
  if (debug) Serial.print(phi2);
  if (debug) Serial.print("\t");
  if (debug) Serial.print(phi3);
  if (debug) Serial.print("\t");
  if (debug) Serial.println(phi4);
  if (debug) Serial.println();

  double theta = theta1;
  double phi = phi1;
  double thrustf = thrust*sqrt(2)/2;

  bool sol1 = theta1 > 0 && theta1 < 180 && phi1 > 0 && phi1 < 180;
  bool sol2 = theta2 > 0 && theta2 < 180 && phi2 > 0 && phi2 < 180;
  bool sol3 = theta3 > 0 && theta3 < 180 && phi3 > 0 && phi3 < 180;
  bool sol4 = theta4 > 0 && theta4 < 180 && phi4 > 0 && phi4 < 180;

  if (sol1) {
    //Serial.println("First Solution");
    theta = theta1;
    phi = phi1;
    thrustf = thrust*sqrt(2)/2;
  } else if (sol2) {
    //Serial.println("Second Solution");
    theta = theta2;
    phi = phi2;
    thrustf = -thrust*sqrt(2)/2;
  } else if (sol3) {
    //Serial.println("Third Solution");
    theta = theta3;
    phi = phi3;
    thrustf = -thrust*sqrt(2)/2;
  } else if (sol4) {
    //Serial.println("Fourth Solution");
    theta = theta4;
    phi = phi4;
    thrustf = thrust*sqrt(2)/2;
  }

  if (debug) Serial.print(theta);
  if (debug) Serial.print("\t");
  if (debug) Serial.println(phi);
  //Serial.println(thrustf);
  
  
   yawServo.write(theta);
   pitchServo.write(phi);
   motor.write(motorCom(thrustf));
 }
    


double MotorMapping::motorCom(double command) {
    //input from -1000, to 1000 is expected from controllers
    double adjustedCom = 1500;
    
    if (abs(command) <= deadband/2) {
        adjustedCom = 1500;
    } else if (command > deadband/2) {
        double xo1 = deadband/2;
        double yo1 = turnOnCom+1500;
        double m1 = (maxCom-yo1)/(1000-xo1);
        adjustedCom = m1*command-m1*xo1+yo1;
    } else if (command < deadband/2) {
        double xo2 = -deadband/2;
        double yo2 = -turnOnCom+1500;
        double m2 = (yo2-minCom)/(xo2-(-1000));
        adjustedCom = m2*command-m2*xo2+yo2;
    } else {
        //should never happen, but write 1500 anyway for safety
        adjustedCom = 1500;
    }
    
    this->motor.write(adjustedCom);
    return adjustedCom;
}
