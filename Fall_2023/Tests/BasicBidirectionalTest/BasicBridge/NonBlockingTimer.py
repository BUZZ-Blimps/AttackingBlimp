from time import time

class NonBlockingTimer:
    def __init__(self, frequency=1, period=None):
        self.lastTime = 0
        self.delay = None

        if period is None and frequency is not None:
                self.delay = 1.0/frequency
        elif period is not None and frequency is None:
                self.delay = period

        if self.delay is None:
            print("NonBlockingTimer improperly initialized! Specify either frequency or period!")
    
    def isReady(self):
        currentTime = time()
        if currentTime - self.lastTime >= self.delay:
            self.lastTime = currentTime
            return True
        else:
            return False