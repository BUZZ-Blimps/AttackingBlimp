#pragma once

#include <Arduino.h>
#include "SerialHandler.h"
#include <functional>

using namespace std;

class UDPHandler{
    public:
        void Init();
        void Update();
        void SendUDP(char flag, String message);

        function<void(String)> callback_UDPRecvMsg;

    private:
        void callback_SerialRecvMsg(String message);
        
        String StringLength(String variable, unsigned int numDigits);

        // ========== Connection parameters ==========
        // const char* wifi_ssid = "Adams iphone";
        // const char* wifi_password = "eeeeeeee";
        // const char* bridgeIP = "172.20.10.14";

        //const char* wifi_ssid = "corelab-superman";
        //const char* wifi_password = "georgesandor";
        //const char* bridgeIP = "10.42.0.158";

        const char* wifi_ssid = "COREBlimp";
        const char* wifi_password = "jollypiano265";
        const char* bridgeIP = "192.168.0.203";

        // const char* UDP_port = "64209";
        const char* UDP_port = "6969";

        const char flag_UDPConnectionData = 'C';
        const char flag_UDPMessage = 'M';
        const char* UDP_identifier = ":)";

        // ========== Variables ==========
        SerialHandler serialHandler;
};


