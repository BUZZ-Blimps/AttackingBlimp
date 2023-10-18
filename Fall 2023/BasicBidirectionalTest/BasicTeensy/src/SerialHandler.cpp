#include "SerialHandler.h"
#include "UDPHandler.h"

void SerialHandler::Init(){
    Serial.begin(115200);
    Serial1.begin(115200);
}

void SerialHandler::Update(){
    // Check for lost serial connection to ESP
    float currentMillis = millis();
    if(connectedToESP && (currentMillis-lastHeartbeatMillis)/1000.0 >= timeout_serial){
        // Lost serial connection to ESP
        connectedToESP = false;
        udpHandler.callback_SerialDisconnect();
    }

    ParseMessages();
}

void SerialHandler::SendSerial(char flag, String message){
    Serial1.print(flag);
    Serial1.print(message);
    Serial1.print(delimiter_serial); // End of message
}

// Parses messages along Serial1 (from ESP)
void SerialHandler::ParseMessages(){
    while(Serial1.available() > 0){
        char currentChar = Serial1.read();
        lastHeartbeatMillis = millis();
        if(currentChar != delimiter_serial){
            totalSerial1 += currentChar;
        }else{
            String message = totalSerial1;
            totalSerial1 = "";
            this->ParseMessage(totalSerial1);
        }
    }
}

void SerialHandler::ParseMessage(String message){
    Serial.print("Received from ESP: \"");
    Serial.print(message);
    Serial.println("\".");

    udpHandler.callback_SerialRecvMsg(message);
}

void SerialHandler::RecordESPHearbeat(){
    lastHeartbeatMillis = millis();
    if(!connectedToESP){
        // New serial connection to ESP
        connectedToESP = true;
        udpHandler.callback_SerialConnect();
    }
}