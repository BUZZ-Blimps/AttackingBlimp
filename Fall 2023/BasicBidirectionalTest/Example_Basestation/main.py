import rclpy
from rclpy.node import Node
import time
from functools import partial

from std_msgs.msg import Float64MultiArray, Bool, String, Float64

class BasestationNode(Node):
    def __init__(self):
        super().__init__('example_basestation')
        self.publisher = self.create_publisher(String, 'testTopic', 1)
        self.create_subscription(Float64, "/Yoshi/TeensyTopic", self.callback_TeensyTopic, 2)
        
        self.timerPeriod = 0.2
        self.timer = self.create_timer(self.timerPeriod, self.callback_timer)

        self.startTime = time.time()

    def callback_timer(self):
        msg = String()
        msg.data = "Hey Teensy! " + str(round(time.time() - self.startTime, 2))
        self.publisher.publish(msg)
        #print("Published: ", msg.data)
    
    def callback_TeensyTopic(self, msg):
        print("Subscribed: ", msg.data)


def main(args=None):
    rclpy.init(args=args)
    basestation = BasestationNode()
    rclpy.spin(basestation)
    rclpy.shutdown()

if __name__ == "__main__":
    main()