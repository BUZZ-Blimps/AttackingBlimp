#include <Teensy_ESP_Comm.h>


// Initialize UDP IP address and port for ESP
void TeensyComms::initUDP(String address, String port)
{
    Serial1.print("I"); // Identify init
    Serial1.print(address); // Send IP address
    Serial1.print(port); // Identify init
    Serial1.print("#"); // Deliminator
}

TeensyComms::TeensyComms()
{
    num_sub = 0; 
}

void TeensyComms::initSubscriber(String topicName, messageType topicType, void *callbackFunctionPtr) // Send information regarding ros2 topics to subscribe to ESP
{
    num_sub += 1; 
    Serial1.print("S");
    Serial1.print(num_sub); // send number of subscriptions
    Serial1.print(topicName); // send topic name
    Serial1.print(topicType); // Send message type 
}


void TeensyComms::publish(String topicName, String topicType, int topicValue, int publishFrequency) // Send information regarding ros2 topics to publish to ESP
{

} 
