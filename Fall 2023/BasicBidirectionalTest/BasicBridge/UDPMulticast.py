import socket
import struct
from threading import Thread
import time
import select

# UDP tutorial: https://wiki.python.org/moin/UdpCommunication

class UDPHelper:
    def __init__(self):
        #self.IP = "192.168.0.200"
        self.IP = "172.20.10.14"
        self.port = 5005

        #Setup UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.IP, self.port))
        self.sock.setblocking(False)
        print("Initialized UDP socket.")

        self.looping = False
        self.callback_UDPRecvMsg = None
        
        #self.targetIP = "172.20.10.2"

    def open(self):
        self.thread = Thread(target=self.loopListen)
        self.thread.start()

    def loopListen(self):
        time.sleep(1)
        print("Listening on UDP socket.")
        self.looping = True
        while(self.looping):
            #time.sleep(0.001)
            self.listen()

    #Checks for message prefix == identifier
    def listen(self):
        #print("waiting to receive message")
        try:
            (data, address) = self.sock.recvfrom(1024)
            #print(data)
            #data = message content
            #address = (ip,port)
        except:
            #print("lmao")
            return
        else:
            message = data.decode(encoding='utf-8', errors='ignore')
            #print(inString)
            if (message[0:2] == ":)"):
                message = message[2:]
                print("Received message \"",message,"\" from ",address,".",sep='')
                IP = address[0]
                self.callback_UDPRecvMsg(IP, message)


    def send(self, IP, message):
        if IP is None:
            return
        message = ":)" + message
        outBytes = message.encode(encoding='utf-8',errors='ignore')
        address = (IP, self.port)
        self.sock.sendto(outBytes, address)
        print("Sending \"",message,"\" to address ",address,sep='')

    def close(self):
        print("UDP socket closing...")
        self.looping = False
        self.thread.join()
        self.sock.close()
        print("UDP socket closed.")