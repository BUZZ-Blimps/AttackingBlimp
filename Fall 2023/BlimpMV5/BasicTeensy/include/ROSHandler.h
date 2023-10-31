#pragma once

#include <Arduino.h>
#include "UDPHandler.h"
#include <functional>
#include <map>
#include "NonBlockingTimer.h"
#include <vector>

using namespace std;

enum MessageType{
    type_Float64MultiArray,
    type_Bool,
    type_String,
    type_Float64,
    type_Int64
};

class ROSHandler{
    public:
        void Init();
        void Update();

        void SubscribeTopic_Float64MultiArray(String topicName, function<void(vector<double>)> callback);
        void SubscribeTopic_Bool(String topicName, function<void(bool)> callback);
        void SubscribeTopic_String(String topicName, function<void(String)> callback);
        void SubscribeTopic_Float64(String topicName, function<void(double)> callback);
        void SubscribeTopic_Int64(String topicName, function<void(int64_t)> callback);

        void PublishTopic_Float64MultiArray(String topicName, vector<double> values);
        void PublishTopic_Bool(String topicName, bool value);
        void PublishTopic_String(String topicName, String value);
        void PublishTopic_Float64(String topicName, double value);
        void PublishTopic_Int64(String topicName, int64_t value);

    private:
        void callback_UDPRecvMsg(String message);

        void SendListSubscribedTopics();

        void SubscribeTopic(String topicName, MessageType topicType, function<void(String)> genericCallback);
        void PublishTopic(String topicName, MessageType topicType, String data);

        void ParseTopic_Float64MultiArray(function<void(vector<double>)> callback, String data);
        void ParseTopic_Bool(function<void(bool)> callback, String data);
        void ParseTopic_String(function<void(String)> callback, String data);
        void ParseTopic_Float64(function<void(double)> callback, String data);
        void ParseTopic_Int64(function<void(int64_t)> callback, String data);

        String StringLength(String variable, unsigned int numDigits);
        String PadInt(int variable, unsigned int numDigits);
        double StringToDouble(String str);
        String DoubleToString(double value);

        const char flag_subscribe = 'S';
        const char flag_publish = 'P';
        const int maxNumDigits_TopicNameLength = 2;

        std::map<String,function<void(String)>> map_genericCallbackFunctions;
        UDPHandler udpHandler;
        NonBlockingTimer timer_sendListSubscribedTopics;
};