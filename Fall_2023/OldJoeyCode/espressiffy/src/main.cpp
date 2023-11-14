#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include "Quack.h"
#include "BlimpClock.h"

// Function Prototypes
void loop();
void send(String message);
void send(String targetID, String flag, String message);
bool readUDP(String* message);
void readPackets();
String packetGetFlag(String packet);
String packetGetMessage(String packet);
void setup();

//const char* ssid = "COREBlimp";
//const char* password = "jollypiano265";

const char* ssid = "Adams iphone";
const char* password = "eeeeeeee";

//IPAddress UDPAddress = IPAddress(192, 168, 0, 200);
IPAddress UDPAddress = IPAddress(172, 20, 10, 14);
const int UDPPort = 5005;

int numMessageTypes = 4;
enum messageType {controllerInput, parameters, debug, newBlimp};
String managerID = "0";
String flags[] = { "I", "P", "D", "N" };
String identifier = ":)";
String localIP;

Quack<String> packets[4];

WiFiUDP UDP;
char incomingPacket[255];  // buffer for incoming packets
char replyPacket[] = "ACK";  // a reply string to send back//unsigned long last_message_recieved = 0;

BlimpClock UDPClock;

BlimpClock heartbeat;

void setup() {
  Serial.begin(115200);

  // Set clock speed
  UDPClock.setFrequency(50);  
  
  // Connect to WIFI
  Serial.printf("Connecting to %s#", ssid);
  WiFi.begin(ssid, password);  
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".#");
  }
 
  Serial.print("Connected!#");
  Serial.printf("Now listening at IP %s, UDP port %d#", WiFi.localIP().toString().c_str(), UDPPort); 
  //UDP.beginMulticast(WiFi.localIP(), UDPAddress, UDPPort);
  UDP.begin(UDPPort);
  UDP.flush();
  
  // Establish ID:
  localIP = WiFi.localIP().toString();

  // Set hearbeat frequency
  heartbeat.setFrequency(1);
}

String total = "";

void loop() {
  if(heartbeat.isReady()){
    send("Hi from ESP.");
  }

  while(Serial.available() > 0){
    char current = Serial.read();
    if(current != '#'){
      total += current;
    }else{
      String message = total;
      total = "";

      send(message);
    }
  }

  String message;
  while(readUDP(&message)){
    Serial.print("ESP received from UDP: \"");
    Serial.print(message);
    Serial.print("\".#");
  }

  return;


  // Check if there is a packet available
  if (UDPClock.isReady()) {
    readPackets();
    Serial.println("CurrentTime="+String(millis()/1000.0));
  }  
  if (heartbeat.isReady()) {
    send("0," + localIP + ":Q:Test");
    //Serial.println("0," + blimpID + ":Q:Test");
  }
}

void send(String message) {
    UDP.beginPacket(UDPAddress, UDPPort);
    String configuredOut = identifier + message;
    UDP.print(configuredOut);
    UDP.endPacket();
    Serial.print("Sending over UDP: \"");
    Serial.print(message);
    Serial.print("\".#");
}

bool readUDP(String* message){
    int parsed = UDP.parsePacket();    
    int avail = UDP.available();
    if(parsed != 0 || avail != 0){
      Serial.print("Parsed (");
      Serial.print(parsed);
      Serial.print(")   Avail (");
      Serial.print(avail);
      Serial.print(")#");
    }
    

    //If nothing available
    if (avail < 1) return false;

    char buffChar;
    char buff[avail + 1];
    
    for (int i = 0; i < avail; i++) {
        UDP.read(&buffChar, 1);
        buff[i] = buffChar;
    }
    
    UDP.flush();    
    buff[avail] = '\0';
    String input = String(buff);
    
    if (input.substring(0, identifier.length()) != identifier){
      return false;
    }else{
      *message = input;
      return true;
    }
}

void readPackets() {
    String packet = "";
    while (readUDP(&packet)) {
      // Skip if empty
      if (packet == "") {
        return;
      }
      
      /*
      String flag = packetGetFlag(packet);
      flag.trim();
      if (flag.equals("B")) continue;      for (int i = 0; i < numMessageTypes; i++) { //Iterate through flags
          if (flag == flags[i]){
            Serial.println("Debug1");
            packets[i].pushTop(packet); //Push packet to quack
          }
      }
      */

      // Relay messages
      Serial.println(packet.length());
      Serial.println(packet);
    }
}

String packetGetFlag(String packet) {
    int firstColon = packet.indexOf(':');
    int secondColon = packet.indexOf(':', firstColon+1);
    return packet.substring(firstColon+1,secondColon);
}

String packetGetMessage(String packet) {
    int firstColon = packet.indexOf(':');
    int secondColon = packet.indexOf(':', firstColon+1);
    return packet.substring(secondColon+1);
}
