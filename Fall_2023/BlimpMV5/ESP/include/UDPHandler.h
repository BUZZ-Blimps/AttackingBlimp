#pragma once

#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <functional>
#include <Arduino.h>
#include <NonBlockingTimer.h>
#include "SerialHandler.h"

using namespace std;

class UDPHandler{
    public:
        void Init();
        void Update();
        void ParseConnectWifi(String message);
        void SendUDP(String message);
        bool checkWifiConnection();

        function<void(String)> callback_UDPRecvMsg;

        SerialHandler* serialHandler = nullptr;

    private:
        void readUDPMessages();
        bool readUDPMessage(String* message);
        void PrintSerialDebug(String message);

        const String identifier = ":)";

        bool connectedWifi = false;

        NonBlockingTimer timer_beginWifi;
        WiFiUDP UDP;

        String wifi_ssid;
        String wifi_password;
        IPAddress bridgeIP;
        int UDP_port;

};