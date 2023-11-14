// #include <ESP8266WiFi.h>
// #include <WiFiUdp.h>
// #include "Quack.h"
// #include "BlimpClock.h"

// const char* ssid = "COREBlimp";
// const char* password = "jollypiano265";

// SoftwareSerial espSerial(1, 0); 

// IPAddress UdpAddress = IPAddress(239, 255, 255, 250);int numMessageTypes = 4;
// enum messageType {controllerInput, parameters, debug, newBlimp};
// String managerID = "0";
// String flags[] = { "I", "P", "D", "N" };
// String identifier = ":)";
// String blimpID;

// Quack<String> packets[4];

// WiFiUDP Udp;
// const int UdpPort = 1900;  // local port to listen on
// char incomingPacket[255];  // buffer for incoming packets
// char  replyPacket[] = "ACK";  // a reply string to send back//unsigned long last_message_recieved = 0;

// BlimpClock UdpClock;
// BlimpClock heartbeat;void setup() {
//   Serial.begin(115200);  
  
//   // Set clock speed
//   UdpClock.setFrequency(50);  
  
//   // Connect to WIFI
//   Serial.printf("Connecting to %s ", ssid);
//   WiFi.begin(ssid, password);  while (WiFi.status() != WL_CONNECTED)
//   {
//     delay(500);
//     Serial.print(".");
//   }
 
//   Serial.println(" connected");
//   Serial.printf("Now listening at IP %s, UDP port %d\n", WiFi.localIP().toString().c_str(), UdpPort);  Udp.beginMulticast(WiFi.localIP(), UdpAddress, UdpPort);
//   Udp.flush();  
  
//   // Establish ID:
//   blimpID = WiFi.localIP().toString();
//   bool identified = false;  
  
//   // Set hearbeat frequency
//   heartbeat.setFrequency(20);
// }

// void loop() {
//   // Check if there is a packet available
//   if (UdpClock.isReady()) {
//     readPackets();
//     Serial.println("CurrentTime="+String(millis()/1000.0))
//   }  if (heartbeat.isReady()) {
//     send("0," + blimpID + ":Q:Test");
//     //Serial.println("0," + blimpID + ":Q:Test");
//   }
// }

// void send(String message) {
//     Udp.beginPacket(UdpAddress, UdpPort);
//     String configuredOut = identifier + message;
//     Udp.print(configuredOut);
//     Udp.endPacket();
// }

// void send(String targetID, String flag, String message) {
//     send(targetID + "," + blimpID + ":" + flag + ":" + message);
// }

// void readPackets() {
//     String packet = "";
//     while (readPacket(&packet)) {
//       // Skip if empty
//       if (packet == "") {
//         return;
//       }
      
//       /*
//       String flag = packetGetFlag(packet);
//       flag.trim();
//       if (flag.equals("B")) continue;      for (int i = 0; i < numMessageTypes; i++) { //Iterate through flags
//           if (flag == flags[i]){
//             Serial.println("Debug1");
//             packets[i].pushTop(packet); //Push packet to quack
//           }
//       }
//       */

//       // Relay messages
//       Serial.println(packet.length());
//       Serial.println(packet);
//     }
// }

// String packetGetFlag(String packet) {
//     int firstColon = packet.indexOf(':');
//     int secondColon = packet.indexOf(':', firstColon+1);
//     return packet.substring(firstColon+1,secondColon);
// }

// String packetGetMessage(String packet) {
//     int firstColon = packet.indexOf(':');
//     int secondColon = packet.indexOf(':', firstColon+1);
//     return packet.substring(secondColon+1);
// }

// bool readPacket(String* packet){
//     int parsed = Udp.parsePacket();    
//     int avail = Udp.available();

//     if (avail < 1) return false; 
    
//     //If nothing available    
//     char buffChar;
//     char buff[avail + 1];
    
//     for (int i = 0; i < avail; i++) {
//         Udp.read(&buffChar, 1);
//         buff[i] = buffChar;
//     }
    
//     Udp.flush();    buff[avail] = '\0';
//     String input = String(buff);    bool valid = true;
    
//     if (input.substring(0, identifier.length()) != identifier) valid = false;
    
//     input = input.substring(identifier.length());
//     int comma = input.indexOf(',');
    
//     if (comma == -1 || input.substring(0, comma) != blimpID) valid = false;
//         if (valid) {
//         *packet = input;
//     }
//     else {
//         *packet = "";
//     }

//     return true;
// }