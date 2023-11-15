#include <Arduino.h>
#include "UDPHandler.h"
#include <functional>

using namespace std;
using namespace std::placeholders;

void UDPHandler::Init(){
    serialHandler.callback_SerialRecvMsg = bind(&UDPHandler::callback_SerialRecvMsg, this, _1);
    serialHandler.Init();
}

void UDPHandler::Update(){
    serialHandler.Update();
}

void UDPHandler::SendUDP(char flag, String message){
    serialHandler.SendSerial(flag_UDPMessage, flag+message);
}

void UDPHandler::callback_SerialRecvMsg(String message){
    char messageFlag = message.charAt(0);
    message = message.substring(1);
    if(messageFlag == flag_UDPConnectionData){
        // UDP connection status
        bool connectedToUDP = (message.charAt(0) != '0');
        Serial.println("ESP connected to WiFi: " + String(connectedToUDP));
        if(!connectedToUDP){
            // No active UDP connection
            // Send UDP connection to ESP
            String initMsg = "";
            initMsg += StringLength(wifi_ssid, 2) + wifi_ssid;
            initMsg += StringLength(wifi_password, 2) + wifi_password;
            initMsg += StringLength(bridgeIP, 2) + bridgeIP;
            initMsg += StringLength(UDP_port, 1) + UDP_port;
            serialHandler.SendSerial(flag_UDPConnectionData, initMsg);
        }else{
            // There is an active UDP connection
        }
    }else if(messageFlag == flag_UDPMessage){
        callback_UDPRecvMsg(message);
    }
}

String UDPHandler::StringLength(String variable, unsigned int numDigits){
    int length = variable.length();
    String lengthStr = String(length);
    unsigned int lengthLengthStr = lengthStr.length();
    if(lengthLengthStr > numDigits){
        // ERROR
        return "ERROR";
    }else{
        // Add zeros
        for(unsigned int i=0; i<(numDigits-lengthLengthStr); i++){
            lengthStr = "0" + lengthStr;
        }
        return lengthStr;
    }
}