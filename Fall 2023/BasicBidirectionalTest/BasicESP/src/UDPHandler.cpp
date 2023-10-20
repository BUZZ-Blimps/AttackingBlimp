#include "UDPHandler.h"
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

void UDPHandler::Init(){
    timer_beginWifi.setPeriod(10);
}

void UDPHandler::Update(){
    // Check Wifi connection status
    if(!connectedWifi && checkWifiConnection()){
        // New wifi connection
        connectedWifi = true;

        UDP.begin(UDP_port);
        UDP.flush();
    }else if(connectedWifi && !checkWifiConnection()){
        // Lost wifi connection
        connectedWifi = false;
    }

    // Receive messages
    readUDPMessages();
}

void UDPHandler::ParseConnectWifi(String message){
    if(checkWifiConnection()) return;
    if(!timer_beginWifi.isReady()) return;

    int index_length_wifi_ssid = 0;
    int length_wifi_ssid = message.substring(index_length_wifi_ssid,index_length_wifi_ssid+2).toInt();
    wifi_ssid = message.substring(index_length_wifi_ssid+2,index_length_wifi_ssid+2+length_wifi_ssid);
    int index_length_wifi_password = index_length_wifi_ssid+2+length_wifi_ssid;
    int length_wifi_password = message.substring(index_length_wifi_password,index_length_wifi_password+2).toInt();
    wifi_password = message.substring(index_length_wifi_password+2,index_length_wifi_password+2+length_wifi_password);
    int index_length_bridgeIP = index_length_wifi_password+2+length_wifi_password;
    int length_bridgeIP = message.substring(index_length_bridgeIP,index_length_bridgeIP+2).toInt();
    bridgeIP.fromString(message.substring(index_length_bridgeIP+2,index_length_bridgeIP+2+length_bridgeIP));
    int index_length_UDP_port = index_length_bridgeIP+2+length_bridgeIP;
    int length_UDP_port = message.substring(index_length_UDP_port,index_length_UDP_port+1).toInt();
    UDP_port = message.substring(index_length_UDP_port+1,index_length_UDP_port+1+length_UDP_port).toInt();

    WiFi.begin(wifi_ssid, wifi_password);
    PrintSerialDebug("Connecting to Wifi.");
}

void UDPHandler::SendUDP(String message){
    UDP.beginPacket(bridgeIP, UDP_port);
    String configuredOut = identifier + message;
    UDP.print(configuredOut);
    UDP.endPacket();
}

bool UDPHandler::checkWifiConnection(){
    return WiFi.status() == WL_CONNECTED;
}

void UDPHandler::readUDPMessages(){
    String message;
    while(readUDPMessage(&message)){
        callback_UDPRecvMsg(message);
    }
}

bool UDPHandler::readUDPMessage(String* message){
    //int parsed = UDP.parsePacket();
    UDP.parsePacket();
    int avail = UDP.available();
    
    //If nothing available
    if (avail < 1) return false;

    char buffChar;
    char buff[avail + 1];
    for (int i = 0; i < avail; i++) {
        UDP.read(&buffChar, 1);
        buff[i] = buffChar;
    }
    
    UDP.flush();    
    buff[avail] = '\0';
    String input = String(buff);

    if(input.length() < identifier.length()) return false;
    if(input.substring(0, identifier.length()) != identifier) return false;

    *message = input.substring(identifier.length());
    return true;
}

void UDPHandler::PrintSerialDebug(String message){
    if(serialHandler != nullptr){
        serialHandler->SendSerial(flag_Debug, message);
    }
}