from UDPMulticast import UDPHelper
from BlimpNode import BlimpNode

class Bridge:
    def __init__(self):
        # UDPHelper
        self.udpHelper = UDPHelper()
        self.udpHelper.callback_UDPRecvMsg = self.callback_UDPRecvMsg
        self.udpHelper.open()

        # Maps
        self.map_BlimpNode_IP = {}

    def Update(self):
        pass

    def callback_UDPRecvMsg(self, IP, message):
        if IP not in self.map_BlimpNode_IP:
            # Blimp not previously registered
            self.map_BlimpNode_IP[IP] = BlimpNode(IP)
        
        blimpNode = self.map_BlimpNode_IP[IP]