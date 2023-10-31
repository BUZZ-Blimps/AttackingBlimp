import rclpy
from rclpy.node import Node
import time
from functools import partial
import numpy as np
import matplotlib.pyplot as plt

from std_msgs.msg import Float64MultiArray, Bool, String, Float64

class BasestationNode(Node):
    def __init__(self):
        super().__init__('example_basestation')
        self.publisher = self.create_publisher(Float64, '/timeIn', 1)
        self.create_subscription(Float64, "/Yoshi/timeOut", self.callback_subscribe, 2)

        self.controlTimer = self.create_timer(0.001, self.callback_timer_control)
        self.timer = None

        #self.rates = [1, 5, 10, 20, 40, 60, 80, 100, 150, 200]
        #self.sampleTime = 20 # [seconds]
        #self.waitTime = 5 # [seconds]
        self.rates = [1, 5, 10, 20, 50, 80]
        self.sampleTime = 30
        self.waitTime = 5
        self.timesTimes = []
        self.timesValues = []
        self.startTime = 0
        self.currentSample = 0
        self.saving = True

    def callback_timer_control(self):
        elapsedTime = time.time() - self.startTime
        if elapsedTime < self.sampleTime:
            pass
        elif elapsedTime < self.sampleTime + self.waitTime:
            self.saving = False
            print("Finished trial.")
        else:
            if len(self.timesTimes) == len(self.rates):
                # Done, plot data
                for i in range(len(self.timesTimes)):
                    plt.plot(self.timesTimes[i], self.timesValues[i])
                plt.show()
            self.timesTimes.append(np.array([]))
            self.timesValues.append(np.array([]))
            self.startTime = time.time()
            self.saving = True
            if self.timer is not None:
                self.timer.cancel()
                self.timer = None
            self.timer = self.create_timer(1.0/self.rates[len(self.timesTimes)-1], self.callback_timer)

    def callback_timer(self):
        msg = Float64()
        msg.data = time.time()
        self.publisher.publish(msg)
        print("Published: ", msg.data)
    
    def callback_subscribe(self, msg):
        print("Subscribed: ", msg.data)
        if self.saving:
            if float(msg.data) > 10000:
                self.timesTimes[len(self.timesTimes)-1] = np.append(self.timesTimes[len(self.timesTimes)-1], time.time())
                self.timesValues[len(self.timesTimes)-1] = np.append(self.timesValues[len(self.timesTimes)-1], time.time()-float(msg.data))

def main(args=None):
    rclpy.init(args=args)
    basestation = BasestationNode()
    rclpy.spin(basestation)
    rclpy.shutdown()

if __name__ == "__main__":
    main()