import socket
import struct
from threading import Thread
import time
import select

# UDP tutorial: https://wiki.python.org/moin/UdpCommunication

class UDPHelper:
    def __init__(self):
        self.IP = "192.168.0.200"
        self.port = 5005

        #Setup UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.IP, self.port))
        self.sock.setblocking(False)
        print("Initialized UDP socket.")

        self.looping = False
        self.callback = None

    def open(self):
        self.thread = Thread(target=self.loopListen)
        self.thread.start()
    
    def setCallback(self, callback):
        self.callback = callback

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
            inString = data.decode(encoding='utf-8', errors='ignore')
            #print(inString)
            if (inString[0:2] == ":)"):
                inString = inString[2:]
                print("Received message \"",inString,"\" from ",address,".",sep='')
                try:
                    val = float(inString)
                    if self.callback is not None:
                        self.callback(val)
                        print(type(self.callback))
                except ValueError:
                    #print("Not a float lmao")
                    pass

    def send(self, targetAddress, message):
        message = ":)" + message
        outBytes = message.encode(encoding='utf-8',errors='ignore')
        self.sock.sendto(outBytes, (targetAddress, self.port))
        print("Sending: \"",message,"\"",sep='')

    def close(self):
        print("UDP socket closing...")
        self.looping = False
        self.thread.join()
        self.sock.close()
        print("UDP socket closed.")