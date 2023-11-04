import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from functools import partial
#from Bridge import Bridge
from pydoc import locate

from std_msgs.msg import Float64MultiArray, Bool, String, Float64, Int64

class BlimpNode(Node):
    def __init__(self, IP, name, func_sendTopicToBlimp):
        
        super().__init__(node_name=name, namespace=name)

        print('Spun up BlimpNode with name {}'.format(name))

        self.IP = IP
        self.name = name
        self.func_sendTopicToBlimp = func_sendTopicToBlimp

        self.lastHeartbeat = 0 # [s]

        self.map_topicName_subscriber: dict[str,Subscription] = {}
        self.map_topicName_publisher: dict[str,Publisher] = {}
        self.map_topicTypeInt_topicType = {
            0: Float64MultiArray,
            1: Bool,
            2: String,
            3: Float64,
            4: Int64
        }

        self.topicBufferSize = 3

    def ParseSubscribeMessage(self, message):
        try:
            msgIndex = 0
            numTopics = int(message[msgIndex:msgIndex+2])
            msgIndex += 2
            for i in range(numTopics):
                # Parse message for each topic -> topicName, topicType
                topicNameLength = int(message[msgIndex:msgIndex+2])
                msgIndex += 2
                topicName = message[msgIndex:msgIndex+topicNameLength]
                msgIndex += topicNameLength
                topicTypeInt = int(message[msgIndex:msgIndex+1])
                msgIndex += 1
                self.CheckSubscription(topicName, topicTypeInt)
        except (ValueError):
            print("Corrupt UDP subscription packet, throwing out message.")
    
    def CheckSubscription(self, topicName, topicTypeInt):
        # If subscription doesn't exist, create it
        if topicName not in self.map_topicName_subscriber:
            # Check for valid topicTypeInt
            if topicTypeInt not in self.map_topicTypeInt_topicType:
                print("Invalid topic type subscribed: ",topicName," (",topicTypeInt,")",sep='')
                return
            topicType = self.map_topicTypeInt_topicType[topicTypeInt]

            # Create generic callback with topicName and topicType
            genericCallback = partial(self.callback_Subscription, topicName, topicTypeInt)

            # Create new subscription with generic callback
            print("New subscription type (",topicType,") - name (",topicName,")",sep='')

            # newSubscription = self.create_subscription(topicType, topicName, genericCallback, self.topicBufferSize)
            newSubscription = self.create_subscription(topicType, topicName, genericCallback, self.topicBufferSize)

            # Save new subscription in map
            self.map_topicName_subscriber[topicName] = newSubscription

            print("Node (",self.name,") subscribed to topic (",topicName,")",sep='')
        else:
            print("Node (",self.name,") already subscribed to topic (",topicName,")",sep='')
    
    def test(self, message1):
        print("ROS published:",message1)

    def callback_Subscription(self, topicName, topicTypeInt, message):
        print("ROS published:", message)
        topicType = self.map_topicTypeInt_topicType[topicTypeInt]
        if topicType == Float64MultiArray:
            topicMessage = self.ParseROSMessage_Float64MultiArray(message)
        elif topicType == Bool:
            topicMessage = self.ParseROSMessage_Bool(message)
        elif topicType == String:
            topicMessage = self.ParseROSMessage_String(message)
        elif topicType == Float64:
            topicMessage = self.ParseROSMessage_Float64(message)
        elif topicType == Int64:
            topicMessage = self.ParseROSMessage_Int64(message)

        self.func_sendTopicToBlimp(self, topicName, topicTypeInt, topicMessage)

    def ParseROSMessage_Float64MultiArray(self, message):
        values = message.data
        strMessage = str(len(values)) + ","
        for value in values:
            strMessage += str(value) + ","
        return strMessage

    def ParseROSMessage_Bool(self, message):
        strMessage = "1" if message.data else "0"
        return strMessage

    def ParseROSMessage_String(self, message):
        strMessage = message.data
        return strMessage

    def ParseROSMessage_Float64(self, message):
        strMessage = str(message.data)
        return strMessage

    def ParseROSMessage_Int64(self, message):
        strMessage = str(message.data)
        return strMessage

    def ParsePublishMessage(self, message):
        try:
            topicNameLength = int(message[0:2])
            topicName = message[2:2+topicNameLength]
            topicTypeInt = int(message[2+topicNameLength:2+topicNameLength+1])
            topicType = self.map_topicTypeInt_topicType[topicTypeInt]
            topicData = message[2+topicNameLength+1:]

            # if topicName[0] == '/':
            #     topicNameExt = "/" + self.name + topicName
            # else:
            #     topicNameExt = "/" + self.name + "/" + topicName

            #Topic name will be scoped automatically by the node name (comment for Adam from Willie)
            topicNameExt = topicName

            # If publisher doesn't exist, make it
            if topicName not in self.map_topicName_publisher:
                self.map_topicName_publisher[topicName] = self.create_publisher(topicType, topicNameExt, self.topicBufferSize)
                print("Created publisher (",topicNameExt,") of type ",topicType,sep='')

            publisher = self.map_topicName_publisher[topicName]

            if topicType == Float64MultiArray:
                rosMessage = self.ParseMessage_Float64MultiArray(topicData)
            elif topicType == Bool:
                rosMessage = self.ParseMessage_Bool(topicData)
            elif topicType == String:
                rosMessage = self.ParseMessage_String(topicData)
            elif topicType == Float64:
                rosMessage = self.ParseMessage_Float64(topicData)
            elif topicType == Int64:
                rosMessage = self.ParseMessage_Int64(topicData)
            publisher.publish(rosMessage)

            print("Node (",self.name,") published topic (",topicNameExt,"): ",rosMessage.data,sep='')
            #print("Type:",type(rosMessage))
        except(ValueError):
            print("Corrupted UDP publisher packet, removing data")

    def ParseMessage_Float64MultiArray(self, topicData):
        # Split with comma delimiters
        valueStrings = topicData.split(",")
        # Get rid of first value (number of real values)
        valueStrings = valueStrings[1:]
        # Get rid of empty last value
        if len(valueStrings[len(valueStrings)-1]) == 0:
            valueStrings = valueStrings[0:len(valueStrings)-1]
        
        values = [float(valueStrings[i]) for i in range(len(valueStrings))]
        rosMessage = Float64MultiArray()
        rosMessage.data = values
        return rosMessage

    def ParseMessage_Bool(self, topicData):
        value = (str(topicData) == "1")
        rosMessage = Bool()
        rosMessage.data = value
        return rosMessage
    
    def ParseMessage_String(self, topicData):
        value = topicData
        rosMessage = String()
        rosMessage.data = value
        return rosMessage
    
    def ParseMessage_Float64(self, topicData):
        value = float(topicData)
        rosMessage = Float64()
        rosMessage.data = value
        return rosMessage
    
    def ParseMessage_Int64(self, topicData):
        value = long(topicData)
        rosMessage = Int64()
        rosMessage.data = value
        return rosMessage
