# Protocol Documentation for UDP-ROS2 Bridge

**MISC**:
- UDPHandler telling ESP to create UDP connection:
    - "C" + [2 digit number of length of wifi_ssid] + [wifi_ssid] + [2 digit number of length of wifi_password] + [wifi_password] + [2 digit number of length of bridgeIP] + [bridgeIP] + [1 digit number of length of UDP_port] + [UDP_port]
- Teensy msg to esp starting with "E": contains UDP connection info
- ESP msg to Teensy starting with "E": single digit boolean for UDP connection status (0=not connected, 1=connected)
- Teensy -> ESP Flags
    - C: UDP connection data
    - M: UDP message
- ESP -> Teensy Flags
    - C: UDP connection data
- Teensy -> Bridge Flags
    - S: list of subscribed topics
    - P: a published topic with its value
- Bridge -> Teensy Flags
    - P: a published topic with its value

**Teensy <-Serial-> ESP01 <-UDP-> Bridge <-ROS2-> Basestation**

**Teensy-ESP01 Message Protocol**:
- Use character ```#``` as serial message delimiter
- Teensy tells ESP to initialize UDP connection, until ESP confirms connection to Teensy
    - "I###.###.#.###:####" = "I" + [IP address] + ":" + [port]

**Teensy-ESP01-Bridge Message Protocol**:
- Initializing a connection
    - Whenever the bridge receives a new connection from an ESP01, get its IP, use IP to get intended name of attack blimp, register blimp with ROS
    - When registering a blimp, create a node/namespace for the blimp, maybe subscribe to identify? 
- Publishing topics
    - Teensy send message to Bridge
        - "P" + [2 digit length of topic name] + [topic name] + [1 digit topic type] + [raw value data]
    - Bridge parses topic name, type, value and publishes to ROS network
- Subscribing to topics
    - Teensy occasionally sends list of subscribed topics to Bridge
        - "S" + [2 digit number of subscribed topics] +
        - For each subscribed topic: [2 digit length of topic name] + [topic name] + [1 digit topic type]
    - Bridge keeps track of topics Teensy is subscribed to and registers internal callback functions
    - When topic callback function is called, Bridge sends subscribed topic to Teensy
        - "S" + [2 digit length of topic name] + [topic name] + [1 digit topic type] + [raw value data]
    - Teensy internally parses topic name, type, value and calls appropriate callback function
- Teensy callback function pattern: ```callbackFunc(string topicName, enum topicType, data0, data1, ...)```
- Supported Message Type enums and ID:
    - Float64Array: 0
    ```callbackFunc(string name, enum type, int arraySize, float* arrayPtr)```
        - Array will be **dynamically allocated** and will get deleted after callback function is called. COPY IF NEEDED. 
    - Boolean: 1
    ```callbackFunc(string name, enum type, boolean value)```
    - String: 2
    ```callbackFunc(string name, enum type, string value)```
    - Float64: 3
    ```callbackFunc(string name, enum type, float value)```

**ESP01**:
- Goal: Forwarding messages
- Consider handshakes with Teensy (serial) and Bridge (UDP)
- Functionality
    - Receive Serial -> Send UDP: ```readSerialBuffer() -> sendUDP(string msg)```
        - Read from serial buffer into internal buffer, until serial message delimiter, then send to UDP
    - Receive UDP -> Send Serial
        - Read packet from UDP, send to serial

**Teensy**:
- Goal: Run normal controls for Attack Blimp and communicate with basestation
- Functionality
    - Can initialize UDP connection: ```initUDP(address, port)```
        - tells ESP01 how to init UDP connection to Bridge
    - Can subscribe to topics: ```initSubscriber(topicName, topicType, callbackFunctionPtr Null*)```
        - Adds subscribed-topic to internal list of subscribed topics
        - List of subscribed topics is occasionally sent to Bridge (for robustness)
        - Callback function pointers can take different arguments, so they should be passed in as a Null* and later re-cast to appropriate function header
    - Can publish topics: ```publish(topicName, topicType, topicValue, publishFrequency)```
        - Attempts to immediately publish value
        - publishFrequency <= 0: no limit on publishing speed
        - publishFrequency > 0: block attempts to publish that are too fast

**Bridge**:
- Goal: Serve as bridge between attack blimps (UDP) and basestation (ROS2)
- Functionality
    - Detects new UDP connections from ESP01
        - Registers new blimp as appropriate
    - Listens to topics published by blimp and re-publishes on ROS network
    - Registers subscriber callback functions for blimp's subscribed topics and sends to blimps when received
    - Consider adding functionality to detect attack blimps that power cycle?
        - Handshake?
        - Detect blimp *disconnect* via timeout (delete subscribers and publishers)