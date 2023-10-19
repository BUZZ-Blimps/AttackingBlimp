#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

#include "NonBlockingTimer.h"

const char flag_UDPConnectionData = 'C';
const char flag_UDPMessage = 'M';

String wifi_ssid;
String wifi_password;
IPAddress bridgeIP;
int UDP_port;

float lastWifiBeginAttempt = 0; // [s]
float delayWifiBeginAttempt = 5; // [s]

String bufferSerial_in = "";

void parseUDPConnectionMessage(String message){
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
}

NonBlockingTimer timer_UDPStatus;
NonBlockingTimer timer_ConnectWifi;

void setup(){
  Serial.begin(115200);

  timer_UDPStatus.setFrequency(2);
  timer_ConnectWifi.setPeriod(5);

  while(WiFi.status() != WL_CONNECTED){
    // Read serial
    while(Serial.available() > 0){
      char current = Serial.read();
      if(current != '#'){
        bufferSerial_in += current;
      }else{
        String message = bufferSerial_in;
        bufferSerial_in = "";

        char flag = message.charAt(0);
        message = message.substring(1);
        if(flag == flag_UDPConnectionData){
          if(timer_ConnectWifi.isReady()){
            parseUDPConnectionMessage(message);

            WiFi.begin(wifi_ssid, wifi_password);
          }
        }
      }
    }
  }
  Serial.printf("Connecting to %s#", ssid);
  WiFi.begin(ssid, password);  
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".#");
  }
 
  Serial.print("Connected!#");
  Serial.printf("Now listening at IP %s, UDP port %d#", WiFi.localIP().toString().c_str(), UDPPort); 
  //UDP.beginMulticast(WiFi.localIP(), UDPAddress, UDPPort);
  UDP.begin(UDPPort);
  UDP.flush();
  
  // Establish ID:
  localIP = WiFi.localIP().toString();
}

void ParseMessage(String message){
  char flag = message.charAt(0);
  message = message.substring(1);
  if(flag == flag_UDPConnectionData){
    if((millis()/1000.0-lastWifiBeginAttempt) >= delayWifiBeginAttempt){
      parseUDPConnectionMessage(message);

      WiFi.begin(wifi_ssid, wifi_password);
    }
    
  }
}

void SendSerial(String message){
  Serial.print(message);
  Serial.print('#');
}

void loop(){
  // Read serial messages
  while(Serial.available() > 0){
    char current = Serial.read();
    if(current != '#'){
      bufferSerial_in += current;
    }else{
      String message = bufferSerial_in;
      bufferSerial_in = "";

      ParseMessage(message);
    }
  }

  if(timer_UDPStatus.isReady()){
    Serial.print(flag_UDPConnectionData)
    if(WiFi.status() == WL_CONNECTED){
      Serial.print('1');
    }else{
      Serial.print('0');
    }
    Serial.print('#');
  }
}