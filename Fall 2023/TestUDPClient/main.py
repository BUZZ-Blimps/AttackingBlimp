from UDPMulticast import UDPHelper
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64

class Node_Pub(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher = self.create_publisher(Float64, 'TeensyValue', 1)

    def publishValue(self, value):
        msg = Float64(data=value)
        self.publisher.publish(msg)
        self.get_logger().info("Published value (%f)" % value)

def main(args=None):
    rclpy.init(args=args)
    nodePub = Node_Pub()

    udpHelper = UDPHelper()
    udpHelper.open()
    udpHelper.setCallback(nodePub.publishValue)

    rclpy.spin(nodePub)

    rclpy.shutdown()

if __name__ == "__main__":
    main()