#include <Arduino.h>
#include "UDPHandler.h"

void UDPHandler::Init(){
    serialHandler.Init();
}

void UDPHandler::Update(){
    serialHandler.Update();
}

void UDPHandler::SendUDP(String message){
    serialHandler.SendSerial(flag_UDPMessage, message);
}

void UDPHandler::callback_SerialRecvMsg(String message){
    char messageFlag = message.charAt(0);
    if(messageFlag == flag_UDPConnectionData){
        // UDP connection status
        bool connectedToUDP = (message.charAt(1) == '1');
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
        
    }
}

void UDPHandler::callback_SerialConnect(){

}

void UDPHandler::callback_SerialDisconnect(){

}

String UDPHandler::StringLength(String variable, int numDigits){
    int length = variable.length();
    String lengthStr = String(length);
    if(lengthStr.length() > numDigits){
        // ERROR
        return "ERROR";
    }else{
        // Add zeros
        for(int i=0; i<(numDigits-lengthStr.length()); i++){
            lengthStr = "0" + lengthStr;
        }
        return lengthStr;
    }
}