#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

WiFiUDP UDP;

const char* wifi_ssid = "Adams iphone";
const char* wifi_password = "eeeeeeee";
IPAddress bridgeIP(172,20,10,14);
int UDP_port = 64209;

void setup() {
  Serial.begin(115200);

  // Connect to Wifi
  WiFi.begin(wifi_ssid, wifi_password);
  Serial.print("DConnecting#");
  while(WiFi.status() != WL_CONNECTED){
    Serial.print("D.#");
    delay(500);
  }
  Serial.print("DConnected!#");

  UDP.begin(UDP_port);
  UDP.flush();
}

void loop() {
  String message = "HiFromTeesny!";
  int a = UDP.beginPacket(bridgeIP, UDP_port);
  String configuredOut = ":)" + message + '\0';
  int b = UDP.write(configuredOut.c_str(),configuredOut.length());
  int c = UDP.endPacket();
  String debugMessage = "M2S1: " + message + ", a=" + a + ", b=" + b + ", c=" + c;
  Serial.print(debugMessage + "#");
  delay(500);
}