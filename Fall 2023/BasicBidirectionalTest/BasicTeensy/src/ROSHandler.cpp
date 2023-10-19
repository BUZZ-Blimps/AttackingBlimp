#include "ROSHandler.h"

using namespace std;
using namespace std::placeholders;

void ROSHandler::Init(){
    udpHandler.callback_UDPRecvMsg = bind(&ROSHandler::callback_UDPRecvMsg, this, _1);
    udpHandler.Init();
}

void ROSHandler::Update(){
    udpHandler.Update();
}

void ROSHandler::SubscribeTopic_Float64MultiArray(String topicName, function<void(int, float*)> callback){
    function<void(String)> genericCallback = bind(&ROSHandler::ParseTopic_Float64MultiArray, this, callback, _1);
    SubscribeTopic(topicName, type_Float64MultiArray, genericCallback);
}

void ROSHandler::SubscribeTopic_Bool(String topicName, function<void(bool)> callback){
    function<void(String)> genericCallback = bind(&ROSHandler::ParseTopic_Bool, this, callback, _1);
    SubscribeTopic(topicName, type_Bool, genericCallback);
}
void ROSHandler::SubscribeTopic_String(String topicName, function<void(String)> callback){
    function<void(String)> genericCallback = bind(&ROSHandler::ParseTopic_String, this, callback, _1);
    SubscribeTopic(topicName, type_String, genericCallback);
}

void ROSHandler::SubscribeTopic_Float64(String topicName, function<void(float)> callback){
    function<void(String)> genericCallback = bind(&ROSHandler::ParseTopic_Float64, this, callback, _1);
    SubscribeTopic(topicName, type_Float64, genericCallback);
}

void ROSHandler::PublishTopic_Float64MultiArray(String topicName, int numValues, float* values){

}

void ROSHandler::PublishTopic_Bool(String topicName, bool value){

}

void ROSHandler::PublishTopic_String(String topicName, String value){

}

void ROSHandler::PublishTopic_Float64(String topicName, float value){

}


void ROSHandler::SubscribeTopic(String topicName, MessageType topicType, function<void(String)> genericCallback){
    String topicIdentifier = topicName + "-" + String(topicType);
    map_genericCallbackFunctions[topicIdentifier] = genericCallback;
}

void ROSHandler::ParseTopic_Float64MultiArray(function<void(int, float*)> callback, String data){
    MessageType messageType = static_cast<MessageType>(data.substring(0,1).toInt());
    if(messageType != type_Float64MultiArray) return;
    data = data.substring(1);

    const char floatDelimiter = ',';

    int numValues = -1;
    float* values = nullptr;
    int nextValueIndex = 0;

    // Ensure data has ending delimiter
    if(data.charAt(data.length()-1) != floatDelimiter) data += floatDelimiter;

    // Iterate through comma delimited float array
    int prevCommaIndex = -1;
    for(int index=0; index<data.length(); index++){
        if(data.charAt(index) == floatDelimiter){
            String currentStr = data.substring(prevCommaIndex+1, index);
            prevCommaIndex = index;
            float currentFloat = currentStr.toFloat();
            if(numValues == -1){
                numValues = (int) currentFloat;
                values = new float[numValues]; // Dynamically allocated memory
            }else{
                values[nextValueIndex] = currentFloat;
                nextValueIndex++;
            }
        }
    }

    // Call callback
    callback(numValues, values);

    // DELETE dynamically alloated memory
    delete[] values; 
}

void ROSHandler::ParseTopic_Bool(function<void(bool)> callback, String data){
    MessageType messageType = static_cast<MessageType>(data.substring(0,1).toInt());
    if(messageType != type_Float64MultiArray) return;
    data = data.substring(1);

    bool value = (data == "0");

    //Call callback
    callback(value);
}

void ROSHandler::ParseTopic_String(function<void(String)> callback, String data){
    MessageType messageType = static_cast<MessageType>(data.substring(0,1).toInt());
    if(messageType != type_Float64MultiArray) return;
    data = data.substring(1);

    String value = data;
    
    //Call callback
    callback(value);
}

void ROSHandler::ParseTopic_Float64(function<void(float)> callback, String data){
    MessageType messageType = static_cast<MessageType>(data.substring(0,1).toInt());
    if(messageType != type_Float64MultiArray) return;
    data = data.substring(1);

    float value = data.toFloat();
    
    //Call callback
    callback(value);
}
