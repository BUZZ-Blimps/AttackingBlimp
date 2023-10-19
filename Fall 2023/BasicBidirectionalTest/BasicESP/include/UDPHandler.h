#pragma once

#include <Arduino>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <functional>

using namespace std;

class UDPHandler{
    public:
        void Init();
        void Update();
        void ParseConnectWifi(String message);
        void SendUDP(String message);
        bool checkWifiConnection();

        function<void(String)> callback_UDPRecvMsg;

    private:
        const String identifier = ":)";

        bool connectedWifi = false;

        String wifi_ssid;
        String wifi_password;
        IPAddress bridgeIP;
        int UDP_port;

};