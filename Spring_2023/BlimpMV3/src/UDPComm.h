#pragma once
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Arduino.h>
#include "Quack.h"
#include <vector>


class UDPComm {
private:
	const char* ssid = "COREBlimp";
	const char* password = "jollypiano265";
	IPAddress udpAddress = IPAddress(239, 255, 255, 250);
	const int udpPort = 1900;
	String identifier = ":)";
	WiFiUDP udp;

	String blimpID;

public:
	Quack<String> packets[4];

	void init();
  	void establishComm();
	String getIPAddress();
	//String getBlimpID(); // DEPRECATED - use getIPAddress() instead

	void send(String message);
	void send(String targetID, String flag, String message);

	bool readPacket(String* packet);
	void readPackets();
  	String packetMove;
  	unsigned long last_message_recieved = 0;

	String packetGetTargetID(String packet);
	String packetGetSourceID(String packet);
	String packetGetFlag(String packet);
	String packetGetMessage(String packet);
  	std::vector<String> packetMoveGetInput();
};
