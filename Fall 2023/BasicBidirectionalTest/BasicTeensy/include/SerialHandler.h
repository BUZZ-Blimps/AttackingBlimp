#pragma once
#include <Arduino.h>
#include "UDPHandler.h"

class SerialHandler{
    public:
        void Init();
        void Update();
        void SendSerial(char flag, String message);

        UDPHandler udpHandler;

    private:
        void ParseMessages();
        void ParseMessage(String message);
        void RecordESPHearbeat();

        const float timeout_serial = 1; // [s]
        const char delimiter_serial = '#';

        // ========== Variables ==========
        bool connectedToESP = false;
        float lastHeartbeatMillis = 0; // [ms]
        String totalSerial1 = "";

};