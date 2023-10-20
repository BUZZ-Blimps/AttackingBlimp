#include <Arduino.h>
#include "SerialHandler.h"
#include "UDPHandler.h"
#include "NonBlockingTimer.h"

SerialHandler serialHandler;
UDPHandler udpHandler;

NonBlockingTimer timer_wifiStatus;
float checkWifiStatusPeriod = 2; // [s]

void callback_SerialRecvMsg(String message);
void callback_UDPRecvMsg(String message);

void setup(){
  serialHandler.callback_SerialRecvMsg = callback_SerialRecvMsg;
  udpHandler.callback_UDPRecvMsg = callback_UDPRecvMsg;

  serialHandler.Init();
  udpHandler.Init();

  udpHandler.serialHandler = &serialHandler;

  timer_wifiStatus.setPeriod(checkWifiStatusPeriod);
}

void callback_SerialRecvMsg(String message){
  char flag = message.charAt(0);
  message = message.substring(1);

  if(flag == flag_UDPConnectionData){
    // Message contains wifi connection details
    udpHandler.ParseConnectWifi(message);

  }else if(flag == flag_UDPMessage){
    // Message should be passed to UDP
    udpHandler.SendUDP(message);
  }
}

void callback_UDPRecvMsg(String message){
  // Message should be passed to serial
  serialHandler.SendSerial(flag_UDPMessage, message);
}

void loop(){
  // Update comms
  serialHandler.Update();
  udpHandler.Update();
  
  // Check wifi status
  if(timer_wifiStatus.isReady()){
    String message = udpHandler.checkWifiConnection() ? "1" : "0";
    serialHandler.SendSerial(flag_UDPConnectionData, message);
  }

  //Serial.print("ESP#");
}