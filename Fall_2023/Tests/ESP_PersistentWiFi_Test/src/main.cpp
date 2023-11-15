#include <Arduino.h>
#include <ESP8266WiFi.h>
//#include <WiFiUdp.h>
#include <vector>

using namespace std;

/*
typedef enum {
    WL_NO_SHIELD        = 255,   // for compatibility with WiFi Shield library
    WL_IDLE_STATUS      = 0,
    WL_NO_SSID_AVAIL    = 1,
    WL_SCAN_COMPLETED   = 2,
    WL_CONNECTED        = 3,
    WL_CONNECT_FAILED   = 4,
    WL_CONNECTION_LOST  = 5,
    WL_WRONG_PASSWORD   = 6,
    WL_DISCONNECTED     = 7
} wl_status_t;
*/

String wifi_status_codes[] = {"WL_IDLE_STATUS", "WL_NO_SSID_AVAIL", "WL_SCAN_COMPLETED", "WL_CONNECTED", "WL_CONNECT_FAILED", "WL_CONNECTION_LOST", "WL_WRONG_PASSWORD", "WL_DISCONNECTED"};

const char* wifi_ssid = "COREBlimp";
// const char* wifi_ssid = "COREBlimp_5G_1";
const char* wifi_password = "jollypiano265";
// const char* wifi_ssid = "Laptop3";
// const char* wifi_password = "bobevans";

void setup() {
  Serial.begin(115200);
  delay(1000);

  //Serial.println("Persistent = " + String(WiFi.getPersistent()));
  // WiFi.persistent(true);
  // Serial.println("Persistent = " + String(WiFi.getPersistent()));

  if(true){
    Serial.println("Beginning WiFi");
    WiFi.begin(wifi_ssid, wifi_password);
  }else{
    Serial.println("NOT Beginnnig WiFi");
  }
  
  wl_status_t wifi_status = WL_DISCONNECTED;
  while(wifi_status != WL_CONNECTED){
    delay(10);
    wifi_status = WiFi.status();
    String wifiStatusCode = "Misc";
    if(0 <= wifi_status && wifi_status <= 7) wifiStatusCode = wifi_status_codes[wifi_status];
    Serial.println("WiFi Status: " + wifiStatusCode + " (" + String(wifi_status) + ")");
  }
  Serial.println("Successfully connected! (" + WiFi.localIP().toString() + ")");
}

void loop() {

}