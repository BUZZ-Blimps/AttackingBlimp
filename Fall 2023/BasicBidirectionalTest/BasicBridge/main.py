from UDPMulticast import UDPHelper
import rclpy
from rclpy.node import Node

from std_msgs.msg import String

class Node_Pub(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher = self.create_publisher(String, 'TeensyValue', 1)
        self.timerPeriod = 1
        self.timer = self.create_timer(self.timerPeriod, self.callback_timer)
        self.udpHelper = udpHelper

    def publishValue(self, value):
        msg = String(data=value)
        self.publisher.publish(msg)
        self.get_logger().info("Published value (%s)" % value)

    def callback_timer(self):
        msg = "Hi from Bridge (" + str(self.get_clock().now()) + ")!"
        self.udpHelper.send(msg)

def main(args=None):
    rclpy.init(args=args)

    global udpHelper
    udpHelper = UDPHelper()

    nodePub = Node_Pub()

    udpHelper.open()
    udpHelper.setCallback(nodePub.publishValue)

    rclpy.spin(nodePub)

    rclpy.shutdown()

if __name__ == "__main__":
    main()