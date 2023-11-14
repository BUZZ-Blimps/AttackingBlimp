#include "SerialHandler.h"

void SerialHandler::Init(){
    Serial.begin(115200);
    Serial1.begin(115200);
}

void SerialHandler::Update(){
    // Check for lost serial connection to ESP
    float currentMillis = millis();
    if(connectedSerial && (currentMillis-lastHeartbeatMillis)/1000.0 >= timeout_serial){
        // Lost serial connection to ESP
        connectedSerial = false;
        if(callback_SerialDisconnect) callback_SerialDisconnect();
    }

    // Read in messages
    ParseMessages();

    // Send out messages
    SendMessages();
}

void SerialHandler::SendSerial(char flag, String message){
    // Add message to bufferSerial1_out
    bufferSerial1_out += flag;
    bufferSerial1_out += message;
    bufferSerial1_out += delimiter_serial;
    //Serial.print("Sending to ESP: ");
    //Serial.println(message);
}

// Parses messages along Serial1 (from ESP)
void SerialHandler::ParseMessages(){
    while(Serial1.available() > 0){
        char currentChar = Serial1.read();
        RecordESPHearbeat();
        if(currentChar != delimiter_serial){
            bufferSerial1_in += currentChar;
        }else{
            String message = bufferSerial1_in;
            bufferSerial1_in = "";
            ParseMessage(message);
        }
        
    }
}

void SerialHandler::ParseMessage(String message){
    //Serial.print("Received from ESP: \"");
    //Serial.print(message);
    //Serial.println("\".");

    if(callback_SerialRecvMsg) callback_SerialRecvMsg(message);
}

void SerialHandler::SendMessages(){
    if(bufferSerial1_out.length() == 0) return;

    if(bytesPerMessage <= 0 || messagesPerSecond <= 0){
        // Send entire buffer immediately
        Serial1.print(bufferSerial1_out);
        bufferSerial1_out = "";
    }else{
        // Check if it is time to send another message
        float currentTimeMicros = micros();
        if(currentTimeMicros - lastMessageOutMicros >= microsPerSecond/messagesPerSecond){
            lastMessageOutMicros = currentTimeMicros;

            // Consider if the buffer is shorter than max message length
            int lastIndex = bytesPerMessage < bufferSerial1_out.length() ? bytesPerMessage : bufferSerial1_out.length();
            String message = bufferSerial1_out.substring(0, lastIndex);
            bufferSerial1_out = bufferSerial1_out.substring(lastIndex);
            Serial1.print(message);
            //Serial.print("SentToESP: ");
            //Serial.println(message);
        }
    }
}

void SerialHandler::RecordESPHearbeat(){
    lastHeartbeatMillis = millis();
    if(!connectedSerial){
        // New serial connection to ESP
        connectedSerial = true;
        if(callback_SerialConnect) callback_SerialConnect();
    }
}