#include <tgmath.h>
#include <Arduino.h>
#include <vector>

#include "BerryIMU_v3.h"
#include "EMAFilter.h"
#include "PID.h"
#include "BlimpClock.h"
#include "Servo.h"
#include "MotorMapping.h"
#include "accelGCorrection.h"
#include "BangBang.h"
#include "baro_acc_kf.h"
#include "gyro_ekf.h"
#include "Madgwick_Filter.h"

#include "ROSHandler.h"
#include "NonBlockingTimer.h"


// IMPORTANT: Critical parameters are located in /include/TeensyParams.h 
#include "TeensyParams.h"

using namespace std;

// Initialize Teensy <-> ROS bridge on Teensy
ROSHandler rosHandler;
NonBlockingTimer timer_pub;

enum states {
  searching,
  approach,
}state;

const char* stateNames[] = {IDNAME(searching), IDNAME(approach)};
const char* stateStr[] = {"searching", "approach"};

enum autonomousStates {
  manual,
  autonomous,
  lost,
} autonomousState;

const char* autonomousStatesNames[] = {IDNAME(manual), IDNAME(autonomous), IDNAME(lost)};
const char* autonomousStatesStr[] = {"manual", "autonomous", "lost"};

int targetColor = 0; 
// blue = 0, red = 1

//motor pins
const int LMPIN = 9; // 26 is pin for Left motor object
const int RMPIN =  6; // 27 is pin for Right motor object
const int LSPIN = 2; //14 is pin for Left servo object
const int RSPIN = 4; //12 is pin for Right servo object 

// Pinout

//msg variables
String msgTemp;
double lastMsgTime = -1.0;
bool outMsgRequest = false;

//servo objects/motor objects
MotorMapping motors;

//objects
BerryIMU_v3 BerryIMU;
Madgwick_Filter madgwick;
BaroAccKF kf;
AccelGCorrection accelGCorrection;

// Kalman_Filter_Tran_Vel_Est kal_vel;
// OpticalEKF xekf(DIST_CONSTANT, GYRO_X_CONSTANT, GYRO_YAW_CONSTANT);
// OpticalEKF yekf(DIST_CONSTANT, GYRO_Y_CONSTANT, 0);

GyroEKF gyroEKF;

EMAFilter yawRateFilter;
EMAFilter pitchRateFilter;

EMAFilter pitchAngleFilter;
EMAFilter rollAngleFilter;

//pre process for accel before vertical kalman filter
EMAFilter verticalAccelFilter;

//baro offset computation from base station value
EMAFilter baroOffset;
//roll offset computation from imu
EMAFilter rollOffset;

// PIDs
PID yawRatePID(3,0,0);  
PID pitchRatePID(2.4,0,0);
//adjust  these for Openmv dont change the middle zeros
PID xPos(0.25,0,0);
PID yPos(0.4,0,0);

//WIFI objects
BlimpClock udpClock;
BlimpClock heartbeat;
BlimpClock motorClock;
BlimpClock serialHeartbeat;

BlimpClock rosClock_ceilHeight;
BlimpClock rosClock_cameraMessage;
BlimpClock rosClock_debug;
BlimpClock rosClock_state;

//variables
double feedbackData[FEEDBACK_BUF_SIZE];
bool autoTransition = false;
bool motorsOff = false; //used for safegaurd

double forwardInput = 0;
double yawInput = 0;
double upInput = 0;

double lastOuterLoopTime = 0;

double ceilHeight = 500;

//IMU orentation
float rotation = -90;

//timing global variables for each update loop
float lastSensorFastLoopTick = 0.0;
float lastBaroLoopTick = 0.0;

//base station baro
float baseBaro = 0.0;

//corrected baro
float actualBaro = 0.0;

//sensor data
float pitch = 0;
float yaw = 0;
float roll = 0;

String s = "";

String buffer_serial2;


std::vector<std::vector<double>> detections;
void processSerial(String msg);

// Callbacks for topics
void callback_motors(vector<double> values);
void callback_auto(bool value);
void callback_targetColor(int64_t value);

unsigned long identify_time;

void setup() {
  Serial.begin(115200);
  rosHandler.Init();

  // Initialize motors, gimbals and sensors
  //pre process for accel before vertical kalman filter

  //motor->(pin,deadband,turn on,min,max)
  motors.Init(LSPIN, RSPIN, LMPIN, RMPIN, 5, 50, 1000, 2000,0.3, &rosHandler);

  // Sensors
  BerryIMU.Init();
  madgwick.Init();
  kf.Init();
  accelGCorrection.Init();
  gyroEKF.Init();

  // EMA Filters
  yawRateFilter.Init(0.2);
  pitchRateFilter.Init(0.1);
  pitchAngleFilter.Init(0.2);
  rollAngleFilter.Init(0.2);
  //pre process for accel before vertical kalman filter
  verticalAccelFilter.Init(0.05);
  //baro offset computation from base station value
  baroOffset.Init(0.5);
  //roll offset computation from imu
  rollOffset.Init(0.5);
  
  // Subscriber Setup //

  //rosHandler.SubscribeTopic_String(TEST_SUB, test_callback); // Test subscription
  rosHandler.SubscribeTopic_Float64MultiArray(MULTIARRAY_TOPIC, callback_motors);
  rosHandler.SubscribeTopic_Bool(AUTO_TOPIC, callback_auto);
  rosHandler.SubscribeTopic_Int64(COLOR_TOPIC, callback_targetColor);

  // Publisher
  rosHandler.PublishTopic_String("/identify", BLIMP_ID);
  identify_time = micros();

  //UART Comm (OpenMV)
  HWSERIAL.begin(115200);
  Serial.println("Color tracking program started");
  delay(2000);

  //initializations
  heartbeat.setFrequency(20);

  //initialize the feedback data as 0s
  for (int i=0; i < FEEDBACK_BUF_SIZE; i++) {
      feedbackData[i] = 0;
  }

  motorClock.setFrequency(30);
  serialHeartbeat.setFrequency(1);

  rosClock_ceilHeight.setFrequency(10);
  rosClock_cameraMessage.setFrequency(10);
  rosClock_debug.setFrequency(5);
  rosClock_state.setFrequency(10);
  
  //wait 2 seconds
  delay(2000);

  rosHandler.PublishTopic_String("log", "Teensy booted and connected to network.");
}

/*test_callback
 * Description: Callback intended to test topic receiving from ROS basestation.
                Print message to serial port 
 * Input: std_msg/String from testTopic 
*/
// void test_callback(String message){
//   Serial.print("Received topic message: \"");
//   Serial.print(message);
//   Serial.println("\".");
// }


double tempforward;
double tempyaw;
double tempup;
/*multiarray_callback
 * Description: Callback intended to convert Float64MultiArray messages to motor commands 
 */
void callback_motors(vector<double> values)
{
  if (values.size() == 4){
    tempforward = values[1];
    tempyaw = values[0];
    tempup = values[3];
    forwardInput = values[1];
    yawInput = values[0];
    upInput = values[3];
  }
}

/*autonomous callback
 * Description: Callback intended to convert Float64MultiArray messages to motor commands 
 */
void callback_auto(bool value) {
  int newState = value ? manual : autonomous;
  if (newState == manual && autonomousState == autonomous) {
    std::string msg = std::string("Going Manual for a Bit...");
    rosHandler.PublishTopic_String("log", msg.c_str());
  }
  else if (newState == autonomous && autonomousState == manual) {
    std::string msg = std::string("Activating Auto Mode");
    rosHandler.PublishTopic_String("log", msg.c_str());
  }
  autonomousState = value ? manual : autonomous;
}

void callback_targetColor(int64_t value){
  if (targetColor == 0 && value == 1) {
    rosHandler.PublishTopic_String("log","Target Color changed to Red");
  } else if (targetColor == 1 && value == 0) {
    rosHandler.PublishTopic_String("log","Target Color changed to Blue");
  }
  targetColor = value;
}

vector<float> times;
vector<String> flags;

void flagTime(String msg){
  times.push_back(micros()/1000.0);
  flags.push_back(msg);
}

void flagTimeStart(){
  times.clear();
  flags.clear();

  flagTime("Start");
}

void flagTimePrint(){
  String msg = "";
  for(unsigned int i=0; i<times.size(); i++){
    msg = msg + flags[i] + ":" + roundDouble(times[i]-times[0],4) + ", ";
  }
  Serial.println(msg);
}


void loop() {
  rosHandler.Update();

  if(rosClock_debug.isReady()){
    rosHandler.PublishTopic_String("debug","Time: " + String(roundDouble(millis()/1000.0,2)));
  }

  if(rosClock_state.isReady()){
    rosHandler.PublishTopic_String("state",stateNames[state]);
  }

  unsigned long now = micros();
  if (now - identify_time > 1.0*MICROS_TO_SEC) {
    // Publisher
    rosHandler.PublishTopic_String("/identify", BLIMP_ID);

    identify_time = now;
  }

  //reading Serial2 color coordinates (OpenMV) and pass them to PID
  /*
  if (HWSERIAL.available()) {
    char c = HWSERIAL.read();
    Serial.print(c);
    if (c !='!'){
      s += c; 
    } else {
      //Serial.println(s);
      //parse string s
      processSerial(s);
      //clear string s
      s = "";
    }
  }
  */

  // Funky serial reading protocol, this is neccessary due to some weird bugs with serial1 and serial2 clashing
  //String incomingString = "";  // initialize an empty string to hold the incoming data
  char startDelimiter = '@';   // set the start delimiter to '@'
  char endDelimiter = '!';     // set the end delimiter to '!'

  // OLD IMPLEMENTATION
  /*
  // Clear Serial Buffers
  Serial2.read();
  //Serial1.read();

  while (Serial2.available()) {
    // Serial.print("Entering serial process");
    if (Serial2.find(startDelimiter)) {
      s = Serial2.readStringUntil(endDelimiter);
      s = s.substring(s.indexOf('@')+1);
      //Serial.print("\nSerial data: \n");
      //Serial.println(s);
      //Serial.println(s.length());

      // Ensure no invalid characters or overlapping packets
      if (s.indexOf('@') == -1 && s.indexOf('!') == -1) {
        // Ensure the packet is of the right size
        if ((s.length() < 96) && (s.length() > 80)){
          processSerial(s);
        }
      }

      // Reset s
      s = "";
    }
  }
  */
  // NEW IMPLEMENTATION
  while(Serial2.available() > 0){
    char currentChar = Serial2.read();
    if(currentChar == startDelimiter){
      buffer_serial2 = "";
    }else if(currentChar == endDelimiter){
      String msg = buffer_serial2;
      buffer_serial2 = "";

      //Ensure the packet is of the right size
      if(80 < msg.length() && msg.length() < 96){
        processSerial(msg);
      }
    }else{
      buffer_serial2 += currentChar;
    }
  }

  //reading data from base station

  // Retrieve inputs from packet
  //std::vector<String> inputs = udp.packetMoveGetInput();


  // ************************** IMU LOOP ************************** //
  float dt = micros()/MICROS_TO_SEC-lastSensorFastLoopTick;
  if (dt >= 1.0/FAST_SENSOR_LOOP_FREQ) {
    lastSensorFastLoopTick = micros()/MICROS_TO_SEC;

    //read sensor values and update madgwick
    BerryIMU.IMU_read();
    BerryIMU.IMU_ROTATION(rotation);

    madgwick.Madgwick_Update(BerryIMU.gyr_rateXraw,
                            BerryIMU.gyr_rateYraw,
                            BerryIMU.gyr_rateZraw,
                            BerryIMU.AccXraw,
                            BerryIMU.AccYraw,
                            BerryIMU.AccZraw);

    //get orientation from madgwick
    pitch = madgwick.pitch_final;
    //Serial.println("PITCH:");
    //Serial.println (pitch);
    roll = madgwick.roll_final;
    yaw = madgwick.yaw_final;

    //compute the acceleration in the barometers vertical reference frame
    accelGCorrection.updateData(BerryIMU.AccXraw, BerryIMU.AccYraw, BerryIMU.AccZraw, pitch, roll);

    //run the prediction step of the vertical velecity kalman filter
    kf.predict(dt);
    // xekf.predict(dt);
    // yekf.predict(dt);
    gyroEKF.predict(dt);


    //pre filter accel before updating vertical velocity kalman filter
    verticalAccelFilter.filter(-accelGCorrection.agz);

    //update vertical velocity kalman filter acceleration
    kf.updateAccel(verticalAccelFilter.last);

    //update filtered yaw rate
    yawRateFilter.filter(BerryIMU.gyr_rateZraw);

    //perform gyro update
    gyroEKF.updateGyro(BerryIMU.gyr_rateXraw*3.14/180, BerryIMU.gyr_rateYraw*3.14/180, BerryIMU.gyr_rateZraw*3.14/180);
    gyroEKF.updateAccel(BerryIMU.AccXraw, BerryIMU.AccYraw, BerryIMU.AccZraw);

    //Serial.println(BerryIMU.AccXraw);
    
  } 

  // ************************** BARO LOOP ************************** //
  dt = micros()/MICROS_TO_SEC-lastBaroLoopTick;
  if (dt >= 1.0/BARO_LOOP_FREQ) {
    lastBaroLoopTick = micros()/MICROS_TO_SEC;

    //get most current imu values
    BerryIMU.IMU_read();
    BerryIMU.IMU_ROTATION(rotation);
    
    //update kalman with uncorreced barometer data
    kf.updateBaro(BerryIMU.alt);


    //compute the corrected height with base station baro data and offset
    actualBaro = BerryIMU.alt - baseBaro + baroOffset.last;

    // xekf.updateBaro(CEIL_HEIGHT_FROM_START-actualBaro);
    // yekf.updateBaro(CEIL_HEIGHT_FROM_START-actualBaro);
  }

  // ******************* PACKET RELATED LOGIC ******************* //

  /*
  //DEBUG SEE INPUTS
  Serial.println("Inputs: ");
  for (int i = 0; i < inputs.size(); i++)
  {
    Serial.print(inputs[i]);
    Serial.print(" ");
  }
  Serial.print("\n");

  Serial.println(atof(inputs[1].c_str()));
  */

  // State Logic
  //String t = inputs[0];
  // Serial.println("State " + t);
  //String targetEnemy = "";

  /*
  if (t == "A") {
    autonomousState = autonomous;
    targetEnemy = inputs[1];
  } else if (t == "M"){
    autonomousState = manual;
    targetEnemy = inputs[5];
  } else {
    autonomousState = lost;
  }*/

  // Serial.println(autonomousState);

  // Target Enemy Logic
  /*
  if(targetEnemy == "R"){
    targetColor = 0;
  }else if(targetEnemy == "G"){
    targetColor = 1;
  }else if(targetEnemy == "B"){
    targetColor = 2;
  }
  */

  // Serial.print("Target Color: ");
  // Serial.println(targetColor);

  // ******************* STATE MACHINE ******************* //
  // Manual
  if (autonomousState == manual){

    if (motorClock.isReady()) {
      //State Machine
      //MANUAL
      //Default

      //Serial.println(udp.packetMoveGetInput()[2].c_str());
      
      //stod() was toDouble()
      //forwardInput = atof(inputs[4].c_str());
      //yawInput = atof(inputs[1].c_str());
      //upInput = atof(inputs[2].c_str());

      //safegaurd: if motor reads any command that is greater than 1, shut the motor off!!!

      forwardInput = max(-1.00, min(1.00,forwardInput));
      upInput = max(-1.00, min(1.00,upInput));
      yawInput = max(-1.00, min(1.00,yawInput));

      // if (abs(forwardInput) >1.0 || abs(yawInput)>1.0 || abs(upInput)>1.0){
      //   motorsOff = true;
      //   Serial.println("Invalid motor input!!!!!");
      // }

      // Serial.println("\nMotor inputs: ");
      // Serial.print(forwardInput);
      // Serial.print(",");
      // Serial.print(upInput);
      // Serial.print(",");
      // Serial.print(yawInput);
      // Serial.print("\n");    

      //map controller input to yaw rate
      //Serial.println(yawInput);
      upInput = 500*tempup;
      forwardInput = 500*tempforward;
      yawInput = -tempyaw*120;    //120 degrees per second



      //Serial.println(yawInput);
    }

  // Autonomous
  } else if (autonomousState == autonomous) {

    //AUTONOMOUS
    //Serial.println("auto");
    double outerLoopTime = millis() - lastOuterLoopTime;

    if (outerLoopTime > (1.0/OUTERLOOP)*1000) {
      lastOuterLoopTime = millis();

      //ultrasonic
      // Serial.print("Ceiling Height: ");
      Serial.println(ceilHeight);
    

      //perform decisions
      switch (state) {

        //Search
        case searching:
          if (true) {
            
            yawInput = -20;   //turning rate while searching

            // Serial.print(ceilHeight);
            //check if the height of the blimp is within this range (ft), adjust accordingly to fall in the zone 
            if (ceilHeight > 200) {
              //Serial.println("up");
              upInput = 100*cos(pitch*3.1415/180.0); //or just 100 (without pitch control)
              forwardInput = 100*sin(pitch*3.1415/180.0);
              
            } else if (ceilHeight < 25) {
              //Serial.println("down");
              upInput = -100*cos(pitch*3.1415/180.0);
              forwardInput = -100*sin(pitch*3.1415/180.0);
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

        // Approach
        case approach:
          if (detections.size() == 3 && detections[targetColor].size() == 2 && detections[targetColor][0] < 500) {
              yawInput = xPos.calculate(-detections[targetColor][0], 0, 100);
              upInput = yPos.calculate(-detections[targetColor][1], 0, 100);
              //forwardInput = 350; //approaching thrust (300 as default)
              forwardInput = 0;
          } else {
            state = searching;
          }

        break;

        // Default Case
        default:
        Serial.println("Invalid State");
        break;
      }
    }
  }
  
  // ******************* MOTOR INPUTS ******************* //
  double yawPIDInput = 0.0;
  double deadband = 2.0; //To do

  //Serial.println(state);
  
  yawPIDInput = yawRatePID.calculate(yawInput, yawRateFilter.last, 100);   
  if (abs(yawInput-yawRateFilter.last) < deadband) {
      yawPIDInput = 0;
  } else {
      yawPIDInput = tanh(yawPIDInput)*abs(yawPIDInput);
  }
  
  // If lost, give zero command
  if (autonomousState == lost) {
    motors.update(0,0,0,0);
  }

  //turing the motors off for debugging for second case
  if (autonomousState == lost){
    motors.update(0,0,0,0);
  }else if (MOTORS_OFF == false && motorsOff == false) {
    // Serial.println("\nafter: ");
    // Serial.println(yawInput);
    // Serial.print(",");
    // Serial.print(upInput);
    // Serial.print(",");
    // Serial.println(forwardInput);

    motors.update(0, forwardInput, upInput, yawPIDInput);
  } else {
    motors.update(0,0,0,0);
  }
  motorsOff = false;

  // End Main Loop
}

// process Serial message from the camera
void processSerial(String msg) {
  String cameraMessageTopicName = "cameraMessage";

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
    if(rosClock_cameraMessage.isReady()) rosHandler.PublishTopic_String(cameraMessageTopicName, "Invalid message (1).");
    return;
  }

  if (splitData.size() != 4) {
    if(rosClock_cameraMessage.isReady()) rosHandler.PublishTopic_String(cameraMessageTopicName, "Invalid splitData size.");
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
      if(rosClock_cameraMessage.isReady()) rosHandler.PublishTopic_String(cameraMessageTopicName, "Invalid message (2).");
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
  if(rosClock_ceilHeight.isReady()) rosHandler.PublishTopic_Float64("ceilHeight",ceilHeight);
  // Serial.println(ceilHeight);

  detections.clear();
  detections.push_back(red);
  detections.push_back(green);
  detections.push_back(blue);

  String cameraMessage = "";
  int doubleDecimals = 2;
  if(red.size() != 0){
    cameraMessage = cameraMessage + "Red: (" + roundDouble(red[0],doubleDecimals) + ", " + roundDouble(red[1],doubleDecimals) + ")";
  }
  if(green.size() != 0){
    if(cameraMessage.length() != 0) cameraMessage = cameraMessage + " - ";
    cameraMessage = cameraMessage + "Green: (" + roundDouble(green[0],doubleDecimals) + ", " + roundDouble(green[1],doubleDecimals) + ")";
  }
  if(blue.size() != 0){
    if(cameraMessage.length() != 0) cameraMessage = cameraMessage + " - ";
    cameraMessage = cameraMessage + "Blue: (" + roundDouble(blue[0],doubleDecimals) + ", " + roundDouble(blue[1],doubleDecimals) + ")";
  }
  if(rosClock_cameraMessage.isReady()) rosHandler.PublishTopic_String(cameraMessageTopicName, cameraMessage);
}

