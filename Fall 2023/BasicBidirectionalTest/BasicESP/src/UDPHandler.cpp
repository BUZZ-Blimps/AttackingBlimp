#include "UDPHandler.h"

void UDPHandler::Init(){

}

void UDPHandler::Update(){
    // Check Wifi connection status
    if(!connectedWifi && checkWifiConnection()){
        // New wifi connection
        connectedWifi = true;

        UDP.begin(UDP_port);
        UDP.flush();
    }

    // Receive messages
}

void UDPHandler::ParseConnectWifi(String message){
    if(checkWifiConnection()) return;

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
}

void UDPHandler::SendUDP(String message){
    // Send message
}

bool UDPHandler::checkWifiConnection(){
    return WiFi.status() == WL_CONNECTED;
}