from UDPHelper import UDPHelper
from BlimpNode import BlimpNode
from time import time
from NonBlockingTimer import NonBlockingTimer


class Bridge:
    def __init__(self):
        # UDPHelper
        self.udpHelper = UDPHelper()
        self.udpHelper.callback_UDPRecvMsg = self.callback_UDPRecvMsg
        self.udpHelper.open()

        # Maps
        self.map_IP_BlimpNode: dict[str,BlimpNode] = {}
        self.map_IP_BlimpName: dict[str,str] = {
            "172.20.10.2": "Yoshi"
        }

        self.flag_subscribe = 'S'
        self.flag_publish = 'P'

        self.timer_debugACK = NonBlockingTimer(frequency=5)

    def Update(self):
        if self.timer_debugACK.isReady():
            self.udpHelper.send("172.20.10.2", "ACK")

    def callback_UDPRecvMsg(self, IP, message):
        return
        if IP not in self.map_IP_BlimpName:
            return
        blimpName = self.map_IP_BlimpName[IP]

        if IP not in self.map_IP_BlimpNode:
            # Blimp not previously registered
            self.map_IP_BlimpNode[IP] = BlimpNode(IP, blimpName, self)
        
        blimpNode = self.map_IP_BlimpNode[IP]

        flag = message[0:1]
        message = message[1:]

        if flag == self.flag_subscribe:
            blimpNode.ParseSubscribeMessage(message)
        elif flag == self.flag_publish:
            blimpNode.ParsePublishMessage(message)
    
    def sendTopicToBlimp(self, blimpNode, topicName, topicTypeInt, topicMessage):
        message = StringLength(topicName) + str(topicTypeInt) + topicMessage
        self.udpHelper.send(blimpNode.IP, self.flag_subscribe, message)

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
