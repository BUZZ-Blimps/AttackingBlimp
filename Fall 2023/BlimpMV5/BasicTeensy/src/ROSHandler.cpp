#include "ROSHandler.h"
#include "NonBlockingTimer.h"
#include <stdlib.h>

using namespace std;
using namespace std::placeholders;

void ROSHandler::Init(){
    udpHandler.callback_UDPRecvMsg = bind(&ROSHandler::callback_UDPRecvMsg, this, _1);
    udpHandler.Init();

    timer_sendListSubscribedTopics.setPeriod(1);
}

void ROSHandler::Update(){
    udpHandler.Update();

    // Occasionally send a list of all subscribed topics
    if(timer_sendListSubscribedTopics.isReady()){
        SendListSubscribedTopics();
    }
}

void ROSHandler::SubscribeTopic_Float64MultiArray(String topicName, function<void(vector<double>)> callback){
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

void ROSHandler::SubscribeTopic_Float64(String topicName, function<void(double)> callback){
    function<void(String)> genericCallback = bind(&ROSHandler::ParseTopic_Float64, this, callback, _1);
    SubscribeTopic(topicName, type_Float64, genericCallback);
}

void ROSHandler::SubscribeTopic_Int64(String topicName, function<void(int64_t)> callback){
    function<void(String)> genericCallback = bind(&ROSHandler::ParseTopic_Int64, this, callback, _1);
    SubscribeTopic(topicName, type_Int64, genericCallback);
}

void ROSHandler::PublishTopic_Float64MultiArray(String topicName, vector<double> values){
    int numValues = values.size();
    String data = numValues + ",";
    for(int i=0; i<numValues; i++){
        data += DoubleToString(values[i]) + ",";
    }
    PublishTopic(topicName, type_Float64MultiArray, data);
}

void ROSHandler::PublishTopic_Bool(String topicName, bool value){
    String data = value ? "1" : "0";
    PublishTopic(topicName, type_Bool, data);
}

void ROSHandler::PublishTopic_String(String topicName, String value){
    String data = value;
    PublishTopic(topicName, type_String, data);
}

void ROSHandler::PublishTopic_Float64(String topicName, double value){
    String data = DoubleToString(value);
    PublishTopic(topicName, type_Float64, data);
}

void ROSHandler::PublishTopic_Int64(String topicName, int64_t value){
    String data = String(value);
    PublishTopic(topicName, type_Int64, data);
}

void ROSHandler::callback_UDPRecvMsg(String message){
    char messageFlag = message.charAt(0);
    message = message.substring(1);
    if(messageFlag == flag_publish){
        // Received published topic
        // Parse UDP message into topicName, topicType, topicData
        int topicNameLength = message.substring(0,maxNumDigits_TopicNameLength).toInt();
        int topicNameIndex = maxNumDigits_TopicNameLength;
        int topicTypeIndex = topicNameIndex + topicNameLength;
        int topicDataIndex = topicTypeIndex + 1;

        String topicName = message.substring(topicNameIndex,topicTypeIndex);
        MessageType topicType = static_cast<MessageType>(message.substring(topicTypeIndex,topicDataIndex).toInt());
        String topicData = message.substring(topicDataIndex);

        // Find and call callback function
        String topicID = topicName + String(topicType);
        auto iter = map_genericCallbackFunctions.find(topicID);
        if(iter != map_genericCallbackFunctions.end()){
            // Callback function exists!
            function<void(String)> genericCallback = iter->second;
            genericCallback(topicData);
        }
    }
}

void ROSHandler::SendListSubscribedTopics(){
    int numSubscribedTopics = map_genericCallbackFunctions.size();
    String message = PadInt(numSubscribedTopics,2);
    for(auto iter = map_genericCallbackFunctions.begin(); iter != map_genericCallbackFunctions.end(); iter++){
        String topicID = iter->first;
        String topicName = topicID.substring(0, topicID.length()-1);
        String topicType = topicID.substring(topicID.length()-1);
        message += StringLength(topicName, 2);
        message += topicName;
        message += topicType;
    }
    udpHandler.SendUDP(flag_subscribe, message);
}

void ROSHandler::SubscribeTopic(String topicName, MessageType topicType, function<void(String)> genericCallback){
    String topicID = topicName + String(topicType);
    map_genericCallbackFunctions[topicID] = genericCallback;
    Serial.print("Subscribed to topic (");
    Serial.print(topicName);
    Serial.print(") with type ");
    Serial.print(topicType);
    Serial.println(".");
    // Immediately send a message upon new subscription
    SendListSubscribedTopics();
}

void ROSHandler::PublishTopic(String topicName, MessageType topicType, String data){
    String message = "";
    message += StringLength(topicName,maxNumDigits_TopicNameLength) + topicName;
    message += String(topicType);
    message += data;
    udpHandler.SendUDP(flag_publish, message);
}

void ROSHandler::ParseTopic_Float64MultiArray(function<void(vector<double>)> callback, String data){
    const char floatDelimiter = ',';

    int numValues = -1;
    vector<double> values;
    int nextValueIndex = 0;

    // Ensure data has ending delimiter
    if(data.charAt(data.length()-1) != floatDelimiter) data += floatDelimiter;

    // Iterate through comma delimited float array
    int prevCommaIndex = -1;
    for(unsigned int index=0; index<data.length(); index++){
        if(data.charAt(index) == floatDelimiter){
            String currentStr = data.substring(prevCommaIndex+1, index);
            prevCommaIndex = index;
            double currentValue = StringToDouble(currentStr);
            if(numValues == -1){
                numValues = (int) currentValue;
            }else{
                values[nextValueIndex] = currentValue;
                nextValueIndex++;
            }
        }
    }

    // Call callback
    callback(values);
}

void ROSHandler::ParseTopic_Bool(function<void(bool)> callback, String data){
    bool value = (data == "0");

    //Call callback
    callback(value);
}

void ROSHandler::ParseTopic_String(function<void(String)> callback, String data){
    String value = data;
    
    //Call callback
    callback(value);
}

void ROSHandler::ParseTopic_Float64(function<void(double)> callback, String data){
    double value = StringToDouble(data);

    //Call callback
    callback(value);
}

void ROSHandler::ParseTopic_Int64(function<void(int64_t)> callback, String data){
    int64_t value = strtoll(data.c_str(), nullptr, 10);

    //Call callback
    callback(value);
}

String ROSHandler::StringLength(String variable, unsigned int numDigits){
    return PadInt(variable.length(), numDigits);
}

String ROSHandler::PadInt(int variable, unsigned int numDigits){
    String varStr = String(variable);
    unsigned int lengthVarStr = varStr.length();
    if(lengthVarStr > numDigits){
        // ERROR
        return "ERROR";
    }else{
        // Add zeros
        for(unsigned int i=0; i<(numDigits-lengthVarStr); i++){
            varStr = "0" + varStr;
        }
        return varStr;
    }
}

double ROSHandler::StringToDouble(String str){
    if(str.length()==0) return 0;

    int strIndex = 0;
    double signMult = 1;
    if(str.charAt(0) == '-'){
        signMult = -1;
        strIndex = 1;
    }

    double value = 0;
    bool period = false;
    int decimalPlace = -1;
    for(unsigned int i=strIndex; i<str.length(); i++){
        char currentChar = str.charAt(i);
        if(currentChar == '.'){
            period = true;
        }else if('0' <= currentChar && currentChar <= '9'){
            if(!period){
                value = 10*value + (currentChar - '0');
            }else{
                value += ((currentChar - '0')*pow10(decimalPlace));
                decimalPlace--;
            }
        }else{
            // ERROR
            return 0;
        }
    }

    return value;
}

String ROSHandler::DoubleToString(double value){
    char buff[30];
    sprintf(buff, "%f", value);
    return String(buff);
}