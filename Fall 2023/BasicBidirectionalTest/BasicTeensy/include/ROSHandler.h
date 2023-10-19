#pragma once

#include <Arduino.h>
#include "UDPHandler.h"
#include <functional>
#include <map>

using namespace std;

enum MessageType{
    type_Float64MultiArray,
    type_Bool,
    type_String,
    type_Float64
};

class ROSHandler{
    public:
        void Init();
        void Update();

        void SubscribeTopic_Float64MultiArray(String topicName, function<void(int, float*)> callback);
        void SubscribeTopic_Bool(String topicName, function<void(bool)> callback);
        void SubscribeTopic_String(String topicName, function<void(String)> callback);
        void SubscribeTopic_Float64(String topicName, function<void(float)> callback);

        void PublishTopic_Float64MultiArray(String topicName, int numValues, float* values);
        void PublishTopic_Bool(String topicName, bool value);
        void PublishTopic_String(String topicName, String value);
        void PublishTopic_Float64(String topicName, float value);

    private:
        void callback_UDPRecvMsg(String message);

        void SubscribeTopic(String topicName, MessageType topicType, function<void(String)> genericCallback);

        void ParseTopic_Float64MultiArray(function<void(int, float*)> callback, String data);
        void ParseTopic_Bool(function<void(bool)> callback, String data);
        void ParseTopic_String(function<void(String)> callback, String data);
        void ParseTopic_Float64(function<void(float)> callback, String data);

        const char flag_subscribe = 'S';
        const char flag_publish = 'P';

        std::map<String,function<void(String)>> map_genericCallbackFunctions;
        UDPHandler udpHandler;
};