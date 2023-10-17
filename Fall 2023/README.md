# Documentation for Fall 2023

## How to compile code to the ESP01 chip - Arduino IDE
Required resources:
- DIYMall ESP01
- DIYMall ESP01 Programmer
- Computer with Linux
Steps:
1. Plug ESP01 into ESP01 Programmer into USB drive
2. In Arduino IDE, select port (/dev/ttyUSB#) and board (Generic ESP8266 Module)
  - To install board support:
    - File > Preferences > Additional Boards Manager, copy in:
      ```https://dl.espressif.com/dl/package_esp32_index.json, http://arduino.esp8266.com/stable/package_esp8266com_index.json```
    - Tools > Board > Boards Manager, install esp8266
3. Upload code!

Basic blinky LED code snippet
```
int ledPin = 1;

void setup(){
  pinMode(ledPin, OUTPUT);
}

void loop(){
  digitalWrite(ledPin, HIGH);
  delay(1000);
  digitalWrite(ledPin, LOW);
  delay(1000);
}
```

## How to compile code to the ESP01 chip - PlatformIO
Required resources:
- DIYMall ESP01
- DIYMall ESP01 Programmer
- Computer with Linux
Steps:
1. Plug ESP01 into ESP01 Programmer into USB drive
2. Create/open a PlatformIO project with "esp8266 board support"
3. Specify upload parameters in platformio.ini
  ```
  [env:esp01]
  platform = espressif8266
  board = esp01
  framework = arduino
  upload_resetmethod = nodemcu
  ```
3. Upload code!
