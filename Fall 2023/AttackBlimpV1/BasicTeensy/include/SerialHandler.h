#pragma once
#include <Arduino.h>
#include <functional>

using namespace std;

class SerialHandler{
    public:
        void Init();
        void Update();
        void SendSerial(char flag, String message);

        function<void(String)> callback_SerialRecvMsg;
        function<void()> callback_SerialConnect;
        function<void()> callback_SerialDisconnect;

    private:
        void ParseMessages();
        void ParseMessage(String message);
        void SendMessages();
        void RecordESPHearbeat();

        const unsigned int bytesPerMessage = 20;
        const float messagesPerSecond = 100; // [Hz]
        const float microsPerSecond = 1000000; // [us/s]

        const float timeout_serial = 1; // [s]
        const char delimiter_serial = '#';

        // ========== Variables ==========
        bool connectedSerial = false;
        float lastHeartbeatMillis = 0; // [ms]
        String bufferSerial1_in = "";
        String bufferSerial1_out = "";
        float lastMessageOutMicros = 0; // [us]

};