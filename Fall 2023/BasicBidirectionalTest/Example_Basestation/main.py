import rclpy
from rclpy.node import Node
import time
from functools import partial
import numpy as np
import matplotlib.pyplot as plt

from std_msgs.msg import Float64MultiArray, Bool, String, Float64
from sensor_msgs.msg import Joy

class BasestationNode(Node):
    def __init__(self):
        super().__init__('example_basestation')

        # Create publishers 
        self.motorPub = self.create_publisher(Float64MultiArray, '/Yoshi/motorCommands', 1)
        self.autoPub = self.create_publisher(Bool, "/Yoshi/auto",1)
        self.targetPub = self.create_publisher(Float64,'/Yoshi/target_color',1)

        #Create subscriptions
        self.create_subscription(Joy, "/joy", self.joy_read, 2)
        self.create_subscription(String, "/identify", self.identity_read, 2)
        
        self.controlTimer = self.create_timer(1, self.callback_timer_one_hz)
        self.motorTimer = self.create_timer(0.1, self.motor_timer_callback)

        # Variables
        self.motorMsg = [0.0, 0.0, 0.0, 0.0]
        self.state = Bool()
        self.target = Float64()
        self.prev_auto_button_pos = False
        self.prev_target_button_pos = False

        # self.timer = None
        #self.rates = [1, 5, 10, 20, 40, 60, 80, 100, 150, 200]
        #self.sampleTime = 20 # [seconds]
        #self.waitTime = 5 # [seconds]
        # self.rates = [1, 5, 10, 20, 50, 80]
        # self.sampleTime = 30
        # self.waitTime = 5
        # self.timesTimes = []
        # self.timesValues = []
        # self.startTime = 0
        # self.currentSample = 0
        # self.saving = True

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

    def callback_timer_one_hz(self):
        self.autoPub.publish(self.state)
        self.targetPub.publish(self.target)
        # print("Published: ", self.state.data, self.target.data)

    def motor_timer_callback(self):
        motorMsg = Float64MultiArray()
        motorMsg.data = self.motorMsg
        self.motorPub.publish(motorMsg)
    
    def joy_read(self, msg):
        # print("Subscribed: ", msg.data)
        # if self.saving:
        #     if float(msg.data) > 10000:
        #         self.timesTimes[len(self.timesTimes)-1] = np.append(self.timesTimes[len(self.timesTimes)-1], time.time())
        #         self.timesValues[len(self.timesTimes)-1] = np.append(self.timesValues[len(self.timesTimes)-1], time.time()-float(msg.data))

        # Read motor commands
        self.motorMsg[0] = msg.axes[0]
        self.motorMsg[1] = msg.axes[1]
        self.motorMsg[2] = msg.axes[3]
        self.motorMsg[3] = msg.axes[4]

        # Determine autonomous state
        if (msg.buttons[0]) and (msg.buttons[0] is not self.prev_auto_button_pos):
            self.state.data = not self.state.data

        # Determine target color 
        if (msg.buttons[2]) and (msg.buttons[2] is not self.prev_target_button_pos):
            if self.target.data:
                self.target.data = 0.0
            else:
                self.target.data = 1.0

        self.prev_auto_button_pos = msg.buttons[0]
        self.prev_target_button_pos = msg.buttons[2]

    def identity_read(self,msg):
        print("Blimps: ", msg)


def main(args=None):
    rclpy.init(args=args)
    basestation = BasestationNode()
    rclpy.spin(basestation)
    rclpy.shutdown()

if __name__ == "__main__":
    main()