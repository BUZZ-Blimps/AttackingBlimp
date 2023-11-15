from UDPHelper import UDPHelper
from BlimpNode import BlimpNode
from time import time
from NonBlockingTimer import NonBlockingTimer
from functools import partial
import rclpy
from threading import Lock


class Bridge:
    def __init__(self):
        # UDPHelper
        self.udpHelper = UDPHelper()
        self.udpHelper.callback_UDPRecvMsg = self.callback_UDPRecvMsg
        self.udpHelper.open()

        # Maps
        self.map_IP_BlimpNode: dict[str,BlimpNode] = {}
        self.map_IP_BlimpName: dict[str,str] = {
            "192.168.0.211" : "Yoshi",
            "192.168.0.212" : "Geoph",
            "192.168.0.213" : "ThisGuy"
        }

        self.flag_subscribe = 'S'
        self.flag_publish = 'P'

        self.startTime = time()
        self.timeout_blimpNodeHeartbeat = 5 # [s]

        self.timer_printBlimps = NonBlockingTimer(frequency=1)

        self.createdNewBlimpNode = False
        self.map_IP_NewBlimpNode = {}
        self.mutex_newlyCreatedBlimpNodes = Lock()
        self.mutex_accessBlimpNodes = Lock()
    
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
        with self.mutex_accessBlimpNodes:
            for IP in blimpNodeIPsToRemove:
                blimpNode = self.map_IP_BlimpNode.pop(IP)
                blimpNode.destroy_node()
                print("Time-out detected of node (",blimpNode.name,")",sep='')
        
        # Check for newly created blimps and add them
        if self.createdNewBlimpNode:
            with self.mutex_newlyCreatedBlimpNodes:
                # Iterate through IPs and add copy over new nodes
                for IP in self.map_IP_NewBlimpNode.keys():
                    self.map_IP_BlimpNode[IP] = self.map_IP_NewBlimpNode[IP]
                # Delete old nodes
                self.map_IP_NewBlimpNode.clear()
                self.createdNewBlimpNode = False
        
        # Print list of currently detected blimps
        if self.timer_printBlimps.isReady():
            elapsedTime = round(currentTime - self.startTime,2)
            print("(",elapsedTime,"s) - ",len(self.map_IP_BlimpNode.keys())," blimp(s) connected.",sep='')
            for IP in self.map_IP_BlimpNode.keys():
                blimpNode = self.map_IP_BlimpNode[IP]
                print("\tBlimp ",blimpNode.name," (",IP,")",sep='')
                print("\t\t Subscribed to: ",end='', flush=True)
                for topicName in blimpNode.map_topicName_subscriber.keys():
                    print(topicName,", ",sep='',end='')
                print()
                print("\t\t Publishing: ",end='', flush=True)
                for topicName in blimpNode.map_topicName_publisher.keys():
                    print(topicName,", ",sep='',end='')
                print()

        for IP in list(self.map_IP_BlimpNode):
            blimpNode = self.map_IP_BlimpNode[IP]
            rclpy.spin_once(blimpNode, timeout_sec=0.001)
    
    def createBlimpNode(self, IP, blimpName):
        with self.mutex_newlyCreatedBlimpNodes:
            if IP not in self.map_IP_NewBlimpNode:
                print("Created blimp (",blimpName,")",sep='')
                self.map_IP_NewBlimpNode[IP] = BlimpNode(IP, blimpName, self.sendTopicToBlimp)
                self.createdNewBlimpNode = True
            newBlimpNode = self.map_IP_NewBlimpNode[IP]
        return newBlimpNode

    def callback_UDPRecvMsg(self, IP, message):
        if IP not in self.map_IP_BlimpName:
            return
        blimpName = self.map_IP_BlimpName[IP]

        blimpNode = None
        if IP not in self.map_IP_BlimpNode:
            # Blimp not previously registered
            blimpNode = self.createBlimpNode(IP, blimpName)
        else:
            with self.mutex_accessBlimpNodes:
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
