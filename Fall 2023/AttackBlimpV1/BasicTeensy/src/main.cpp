#include <Arduino.h>
#include "ROSHandler.h"
#include "NonBlockingTimer.h"
#include "TeensyParams.h"

// IMPORTANT: Critical parameters are located in /include/TeensyParams.h 


// Initialize Teensy <-> ROS bridge on Teensy
ROSHandler rosHandler;
NonBlockingTimer timer_pub;


// Callbacks for topics

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


/*multiarray_callback
 * Description: Callback intended to convert Float64MultiArray messages to motor commands 
 */

void cmd_callback(int arraySize, float* arrayPtr)
{
  // Need to determine order of elements in Float64MultiArray  
  
}

void setup(){
  rosHandler.Init();
  
  // Subscriber Setup //

  //rosHandler.SubscribeTopic_String(TEST_SUB, test_callback); // Test subscription

  rosHandler.SubscribeTopic_Float64MultiArray(MULTIARRAY_TOPIC, cmd_callback);




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
    rosHandler.PublishTopic_Float64(TEST_PUB, millis());
  }
}