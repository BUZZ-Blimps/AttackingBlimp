/*  Teensy to and from ESP Comm header File  */
/*

    Description: This header includes the 
    various methods for teensy-esp communication
    over serial. 

*/

#pragma once

#include <Arduino.h>


enum messageType {msgFloat64Multiarray, msgBool, msgString, msgFloat64};


void initUDP(String address, String port); // Initialize UDP IP address and port for ESP
void initSubscriber(String topicName, messageType topicType, void* callbackFunctionPtr); // Send information regarding ros2 topics to subscribe to ESP
void publish(String topicName, String topicType, int topicValue, int publishFrequency); // Send information regarding ros2 topics to publish to ESP