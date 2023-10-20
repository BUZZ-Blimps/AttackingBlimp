class BlimpNode:
    def __init__(self, IP):
        self.IP = IP
        self.subscribers = []
        self.publishers = []