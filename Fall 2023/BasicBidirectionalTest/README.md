# Basic Bidirectional Communications Test
## Summary
This test focuses on the essential functionality of the communications layer between the Teensy 4.0, ESP-01, Bridge, Blimp Node, and ROS Backend.

## Setup
The following steps are to configure and run the basic bidirectional test:

### 1. Configure Network and UDP Port

Before starting the comms layer, network and port settings need to be configured in these places:
    
- [~/BasicBridge/Bridge.py line 18](BasicBridge/Bridge.py)
- [~/BasicBridge/UDPHelper.py line 11]()
- [~/BasicTeensy/include/UDPHandler.h lines 22-36](BasicTeensy/include/UDPHandler.h)

The network and UDP port settings for CoreBlimp in the Mezzanine are as follows:
```
SSID: COREBlimp
Password: jollypiano265
Bridge IP Address: 192.168.0.203
UDP Port: 5010
```



While these should remain constant, it is good practice to check the IP address using the following command in terminal:

```
ifconfig
```


This should produce a message like

```
lo: flags=73<UP,LOOPBACK,RUNNING>  mtu 65536
        inet 127.0.0.1  netmask 255.0.0.0
        inet6 ::1  prefixlen 128  scopeid 0x10<host>
        loop  txqueuelen 1000  (Local Loopback)
        RX packets 74289  bytes 20148857 (20.1 MB)
        RX errors 0  dropped 0  overruns 0  frame 0
        TX packets 74289  bytes 20148857 (20.1 MB)
        TX errors 0  dropped 0 overruns 0  carrier 0  collisions 0

wlp0s20f3: flags=4163<UP,BROADCAST,RUNNING,MULTICAST>  mtu 1500
        inet 192.168.0.203  netmask 255.255.255.0  broadcast 192.168.0.255
        inet6 fe80::15dc:9faa:217:e337  prefixlen 64  scopeid 0x20<link>
        ether a4:f9:33:b9:1e:0a  txqueuelen 1000  (Ethernet)
        RX packets 557510  bytes 332315309 (332.3 MB)
        RX errors 0  dropped 0  overruns 0  frame 0
        TX packets 614351  bytes 134227756 (134.2 MB)
        TX errors 0  dropped 0 overruns 0  carrier 0  collisions 0
```

The IP address needed is under the wlan (abbreviated ```wl```) heading. The proper IP address is preceded by "inet", and, in this case, it is 192.168.0.203.

The variable names in each program should make it obvious which oarameters go where, but if you have questions ask Adam.

### 2. Flash Code to ESP-01 using PlatformIO and the DIYMall Programmer

First, plug the ESP-01 into the DIYMall programmer. Flash the following code to the ESP-01 by opening it using PlatformIO:
- [ESP-01 Code](BasicESP/src/main.cpp)

Occasionally, a permission error may occur.

```[Errno 13] Permission denied: '/dev/ttyUSB0'```

To fix this, paste the following command into the terminal:
```
sudo chmod a+rw /dev/ttyUSB0`
```

### 3. Flash Code to the Teesny using PlatformIO 

Open the following code and upload it to the Teensy using PlatformIO:

- [Teensy Code](BasicTeensy/src/main.cpp)


### 4. Start up the Example Basestation in Terminal

To start the Basestation, navigate to the following (or equivalent) directory in terminal:

```
~/GitHub/AttackingBlimp/Fall\ 2023/BasicBidirectionalTest/Example_Basestation/
```

Once in the correct directory, run the following command in terminal:

```
python3 main,py
```

### 5. Start up the Communcations Bridge in Terminal

To start the bridge, open a separate terminal window. Then, navigate to the following (or equivalent) directory: 
```
~/GitHub/AttackingBlimp/Fall\ 2023/BasicBidirectionalTest/BasicBridge/
```

Once in the correct directory, run the following command in terminal:
```
python3 main,py
```

## Function

Write stuff Joey