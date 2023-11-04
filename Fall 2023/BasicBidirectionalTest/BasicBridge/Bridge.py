from UDPHelper import UDPHelper
from BlimpNode import BlimpNode
from time import time
from NonBlockingTimer import NonBlockingTimer
from functools import partial
import rclpy

class Bridge:
    def __init__(self):
        # UDPHelper
        self.udpHelper = UDPHelper()
        self.udpHelper.callback_UDPRecvMsg = self.callback_UDPRecvMsg
        self.udpHelper.open()

        # Maps
        self.map_IP_BlimpNode: dict[str,BlimpNode] = {}
        self.map_IP_BlimpName: dict[str,str] = {
            "192.168.0.100": "Yoshi"
        }

        self.flag_subscribe = 'S'
        self.flag_publish = 'P'

        self.startTime = time()
        self.timeout_blimpNodeHeartbeat = 5 # [s]

        self.timer_printBlimps = NonBlockingTimer(frequency=1)
    
    def __del__(self):
        self.udpHelper.close()
    
    def close(self):
        self.__del__()

    def Update(self):
        currentTime = time()

        # Check for timed-out blimps and delete node
        blimpNodeIPsToRemove = []        
        for IP in self.map_IP_BlimpNode.keys():
            blimpNode = self.map_IP_BlimpNode[IP]
            if currentTime - blimpNode.lastHeartbeat >= self.timeout_blimpNodeHeartbeat:
                # Time out has occured, mark node for deletion
                blimpNodeIPsToRemove.append(IP)
        # Actually delete nodes (can't iterate through map AND delete elements of map at same time)
        for IP in blimpNodeIPsToRemove:
            blimpNode = self.map_IP_BlimpNode.pop(IP)
            blimpNode.destroy_node()
            print("Time-out detected of node (",blimpNode.name,")",sep='')
        
        # Print list of currently detected blimps
        if self.timer_printBlimps.isReady():
            elapsedTime = round(currentTime - self.startTime,2)
            print("(",elapsedTime,"s) - ",len(self.map_IP_BlimpNode.keys())," blimp(s) connected.",sep='')
            for IP in self.map_IP_BlimpNode.keys():
                blimpNode = self.map_IP_BlimpNode[IP]
                print("\tBlimp ",blimpNode.name," (",IP,")",sep='')
                if len(blimpNode.map_topicName_subscriber.keys()) > 0:
                    print("\t\t Subscribed to: ",end='')
                    for topicName in blimpNode.map_topicName_subscriber.keys():
                        print(topicName,", ",sep='',end='')
                    print()
                if len(blimpNode.map_topicName_publisher.keys()) > 0:
                    print("\t\t Publishing: ",end='')
                    for topicName in blimpNode.map_topicName_publisher.keys():
                        print(topicName,", ",sep='',end='')
                    print()
                    
        # Spin all blimp nodes
        for IP in self.map_IP_BlimpNode.keys():
            blimpNode = self.map_IP_BlimpNode[IP]
            rclpy.spin_once(blimpNode)

    def callback_UDPRecvMsg(self, IP, message):
        if IP not in self.map_IP_BlimpName:
            return
        blimpName = self.map_IP_BlimpName[IP]

        if IP not in self.map_IP_BlimpNode:
            # Blimp not previously registered
            #func_sendTopicToBlimp = partial(self.sendTopicToBlimp, self)
            self.map_IP_BlimpNode[IP] = BlimpNode(IP, blimpName, self.sendTopicToBlimp)
        
        blimpNode = self.map_IP_BlimpNode[IP]
        blimpNode.lastHeartbeat = time()

        flag = message[0:1]
        message = message[1:]

        if flag == self.flag_subscribe:
            blimpNode.ParseSubscribeMessage(message)
        elif flag == self.flag_publish:
            blimpNode.ParsePublishMessage(message)
    
    def sendTopicToBlimp(self, blimpNode, topicName, topicTypeInt, topicMessage):
        message = StringLength(topicName,2) + topicName + str(topicTypeInt) + topicMessage
        self.udpHelper.send(blimpNode.IP, self.flag_publish, message)

def StringLength(variable, numDigits):
    length = len(variable)
    lengthStr = str(length)
    lengthLengthStr = len(lengthStr)
    if lengthLengthStr > numDigits:
        return "ERROR"
    else:
        for i in range(numDigits-lengthLengthStr):
            lengthStr = "0" + lengthStr
        return lengthStr
