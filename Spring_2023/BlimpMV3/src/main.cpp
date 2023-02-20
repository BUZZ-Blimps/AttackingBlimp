#include <tgmath.h>

#include "IMU_Control.h"
#include "EMAFilter.h"
#include "PID.h"
#include "BlimpClock.h"
#include "UDPComm.h"
#include "ESP32Servo.h"
#include "MotorMapping.h"

//this is the 3 feedbacks
#define FEEDBACK_BUF_SIZE 3

#define MOTORS_OFF false //used for debugging

#define RXD2 16
#define TXD2 17

#define OUTERLOOP 10  //Hz

#define IDNAME(name) #name

enum states {
  searching,
  approach,
}state;

const char* stateNames[] = {IDNAME(searching), IDNAME(approach)};
const char* stateStr[] = {"searching", "approach"};

enum autonomousStates {
  autonomous,
  manual,
  lost,
}autonomousState;

const char* autonomousStatesNames[] = {IDNAME(autonomous), IDNAME(manual), IDNAME(lost)};
const char* autonomousStatesStr[] = {"autonomous", "manual", "lost"};

int targetColor = 2; 
//r 0, g 1, b 2

//motor pins
const int LMPIN = 26; // 26 is pin for Left motor object
const int RMPIN =  27; // 27 is pin for Right motor object
const int LSPIN = 14; //14 is pin for Left servo object
const int RSPIN = 12; //12 is pin for Right servo object 

//servo objects/motor objects

//motor->(pin,deadband,turn on,min,max)
MotorMapping motors(LSPIN, RSPIN, LMPIN, RMPIN, 5, 50, 1000, 2000,0.3);

IMU_Control imu;

EMAFilter yawRateFilter(0.2);
EMAFilter pitchRateFilter(0.1);

EMAFilter pitchAngleFilter(0.2);
EMAFilter rollAngleFilter(0.2);

PID yawRatePID(3,0,0);  
PID pitchRatePID(2.4,0,0);


//adjust  these for Openmv dont change the middle zeros
PID xPos(0.25,0,0);

PID yPos(0.5,0,0);

//WIFI objects
BlimpClock udpClock;
BlimpClock heartbeat;
BlimpClock motorClock;
UDPComm udp;

//variables
double feedbackData[FEEDBACK_BUF_SIZE];
bool autoTransition = false;
bool motorsOff = false; //used for safegaurd

double forwardInput = 0;
double yawInput = 0;
double upInput = 0;

double lastOuterLoopTime = 0;

double ceilHeight = 500;

String s = "";


std::vector<std::vector<double>> detections;
void processSerial(String msg);


void setup() {
  
  // put your setup code here, to run once:
  Serial.begin(115200);

  //UART Comm (OpenMV)
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
  delay(1000);
  Serial.println("Color tracking program started");

    //initializations
    udpClock.setFrequency(100);
    udp.init(); 
    udp.establishComm();
    heartbeat.setFrequency(20);
    

    //initialize the feedback data as 0s
    for (int i=0; i < FEEDBACK_BUF_SIZE; i++) {
        feedbackData[i] = 0;
    }

    motorClock.setFrequency(400);

    imu.IMU_Setup(); 

  
  //wait 2 seconds
  delay(2000);
}

void loop() {
  // put your main code here, to run repeatedly:
  
  //reading Serial2 color coordinates (OpenMV) and pass them to PID
  if (Serial2.available()>0){
    char c = Serial2.read();
    //Serial.print(c);
    if (c !='!'){
      s += c; 
    } else {
      //parse string s
      processSerial(s);
      //clear string s
      s = "";
    }
  }

  //reading data from base station
  if (udpClock.isReady()) {
        udp.readPackets();
        //packets are read and received
        //Read parameters
    
        //sending message to the station
    if (heartbeat.isReady()) {
        feedbackData[0] = autonomous;
        String feedbackMessage = "";
        feedbackMessage += FEEDBACK_BUF_SIZE;
        feedbackMessage += "=";
        for (int i = 0; i < FEEDBACK_BUF_SIZE; i++) {
            feedbackMessage += feedbackData[i];
            feedbackMessage += ",";
        }
        udp.send("0","P",feedbackMessage);
    }

    //fetch imu data
    imu.update();

    //get and filter yaw rate, pitch angle
    double yawRate = yawRateFilter.filter(imu.getYawRate());
    double pitchRate = pitchRateFilter.filter(imu.getPitchRate());
    double pitchAngle = pitchAngleFilter.filter(imu.getPitch());
    double rollAngle = rollAngleFilter.filter(imu.getRoll())-90;

    //printing imu data 
    /*
    Serial.println(pitchRate);
    Serial.print("\t");
    Serial.print(yawRate);
    Serial.print("\t");
    Serial.print(pitchAngle);
    Serial.print("\t");
    Serial.println(rollAngle);
    */

    /*
    WebSerial.print(yawRate);
    WebSerial.print("\t");
    WebSerial.print(pitchAngle);
    WebSerial.print("\t");
    WebSerial.println(rollAngle);
    */

   
    //update motors according to the inputs from the station/joysticks
    std::vector<String> inputs = udp.packetMoveGetInput();
    char t = udp.packetMoveGetInput()[0][0]; //state
    String targetEnemy = "";
    if ((float)udp.last_message_recieved <= (float)millis() - (float)5000) {
      autonomousState = lost;
    } else if (t == 'A') {
      autonomousState = autonomous;
      targetEnemy = inputs[1];
    } else if (t == 'M'){
      autonomousState = manual;
      targetEnemy = inputs[5];
    }

    if(targetEnemy == "R"){
      targetColor = 0;
    }else if(targetEnemy == "G"){
      targetColor = 1;
    }else if(targetEnemy == "B"){
      targetColor = 2;
    }

    //state machine
    if (autonomousState == manual){
        if (motorClock.isReady()) {
          //State Machine
          //MANUAL
          //Default
          
          forwardInput = udp.packetMoveGetInput()[4].toDouble();
          yawInput = udp.packetMoveGetInput()[1].toDouble();
          upInput = udp.packetMoveGetInput()[2].toDouble();

          //safegaurd: if motor reads any command that is greater than 1, shut the motor off!!!
          if (abs(forwardInput) >1.0 || abs(yawInput)>1.0 || abs(upInput)>1.0){
            motorsOff = true;
            Serial.println("Invalid motor input!!!");
          }
            
//            Serial.println(forwardInput);
//            Serial.println(steerInput);
//            Serial.println(upInput);
           
          //map controller input to yaw rate
          //Serial.println(yawInput);
          upInput = 500*upInput;
          forwardInput = 500*forwardInput;
          yawInput = -yawInput*120;    //120 degrees per second
          //Serial.println(yawInput);
        }
      } else if (autonomousState == autonomous) {
      //AUTONOMOUS
      //Serial.println("auto");
      double outerLoopTime = millis() - lastOuterLoopTime;
      if (outerLoopTime > (1.0/OUTERLOOP)*1000) {
        lastOuterLoopTime = millis();
        //Serial.println("Loop");
        /*
        //EXAMPLE
        //print coordinates of the detection points
        if (detections.size() == 3) {
          for (int i = 0; i < 3; i++) {
            if (i == 0) {
              Serial.println("Red");
            } else if (i == 1) {
              Serial.println("Green");
            } else {
              Serial.println("Blue");
            }
            
            if (detections[i].size() == 2 && abs(detections[i][0]) < 500) {
              Serial.print("X: ");
              Serial.print(detections[i][0]);
              Serial.print("\tY: ");
              Serial.println(detections[i][1]);
            } else {
              Serial.println();
            }
          }
        }
        */
        //END OF EXAMPLE
                
        //ultrasonic
        Serial.print("Ceiling Height: ");
        Serial.println(ceilHeight);
       

        //perform decisions!!!!!!!!!
        switch (state) {
          case searching:
            if (true) {
              
              yawInput = -20;   //turning rate while searching


              //check if the height of the blimp is within this range (ft), adjust accordingly to fall in the zone 
              if (ceilHeight > 120) {
                Serial.println("up");
                upInput = 100*cos(pitchAngle*3.1415/180.0);
                forwardInput = 100*sin(pitchAngle*3.1415/180.0);
                
              } else if (ceilHeight < 75) {
                Serial.println("down");
                upInput = -100*cos(pitchAngle*3.1415/180.0);
                forwardInput = -100*sin(pitchAngle*3.1415/180.0);
              } else {
                upInput = 0;
                forwardInput = 0;
              }
              
              //we see something o_O
              if (detections.size() == 3 && detections[targetColor].size() == 2 && detections[targetColor][0] < 500) {
                state = approach;
              }
              
            }
          break;
          case approach:
            if (detections.size() == 3 && detections[targetColor].size() == 2 && detections[targetColor][0] < 500) {
                yawInput = xPos.calculate(-detections[targetColor][0], 0, 100);
                upInput = yPos.calculate(-detections[targetColor][1], 0, 100);
                forwardInput = 300; //approaching thrust (300 as defalt)
            } else {
              state = searching;
            }
          break;
          default:
          Serial.println("Invalid State");
          break;
        }
        
      }
    }
//        Serial.println("previous");
//        Serial.println(yawInput);
//        Serial.println("yaw rate");
//        Serial.println(yawRate);
         //PID controller (desired, setpoint, sampling rate)
         //double pitch = pitchRatePID.calculate(0, pitchRate, 100);
         
         double yawPIDInput = 0.0;
         double deadband = 2.0; //To do
         
         yawPIDInput = yawRatePID.calculate(yawInput, yawRate, 100);   
         if (abs(yawInput-yawRate) < deadband) {
             yawPIDInput = 0;
         } else {
             yawPIDInput = tanh(yawPIDInput)*abs(yawPIDInput);
         }
         
        if (autonomousState == lost) {
          motors.update(0,0,0,0);
        }
        //turing the motors off for debugging for second case
        else if (autonomousState == lost)
        {
          motors.update(0,0,0,0);
        }
        else if (MOTORS_OFF == false && motorsOff == false) {
//          Serial.println("after");
//          Serial.println(yawInput);
//          Serial.print(",");
//          Serial.print(upInput);
//          Serial.print(",");
//          Serial.println(forwardInput);
//            motors.update(0,forwardInput,upInput,yawPIDInput);
          motors.update(0, forwardInput, upInput, yawPIDInput);
        } else {
          motors.update(0,0,0,0);
        }
  }
}

// process Serial message from the camera
void processSerial(String msg) {
    std::vector<double> green;
    std::vector<double> blue;
    std::vector<double> red;

    std::vector<String> splitData;
    std::vector<String> object;
    //Serial.print("Message");
    //split data
    if (msg.length() > 2) {
    int first = 0;
    int index = 1;
    while (index < msg.length()) {
      if (msg[index] == ';') {
        String k = msg.substring(first, index);
        k.trim();
        splitData.push_back(k);
        first = index+1;
        index = first+1;
      }
      index += 1;
    }
  } else {
    //invalid message
    Serial.println("Invalid Message From Open MV");
    return;
  }

  if (splitData.size() != 4) {
    return;
  }

  for (int i = 0; i < 3; i++) {
    //Serial.println(splitData[i]);
    
    if (splitData[i].length() > 2) {
      int first = 0;
      int index = 1;
      while (index < splitData[i].length()) {
        if (splitData[i][index] == ',') {
          String k = splitData[i].substring(first, index);
          k.trim();
          object.push_back(k);
          first = index+1;
          index = first+1;
        }
        index += 1;
      }
    } else {
      //invalid message
      Serial.println("Invalid Message From Open MV");
      return;
    }

    //get x,y coordinates
    double x = object[0].toFloat()-320/2;
    double y = -(object[1].toFloat()-240/2);

    if (object.size() == 3) {
      switch (object[2][0]) {
        case 'r':
        red.push_back(x);
        red.push_back(y);
        break;
        case 'g':
        green.push_back(x);
        green.push_back(y);
        break;
        case 'b':
        blue.push_back(x);
        blue.push_back(y);
        break;
        default:
        //do noting, invalid color option
        Serial.println("Invalid Color Option");
        break;
      }
    }

    object.clear();
  }

  ceilHeight = (double)splitData[3].toFloat();


  detections.clear();
  detections.push_back(red);
  detections.push_back(green);
  detections.push_back(blue);
}