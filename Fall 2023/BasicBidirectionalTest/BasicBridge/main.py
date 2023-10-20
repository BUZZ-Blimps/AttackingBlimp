from UDPHelper import UDPHelper
import rclpy
from rclpy.node import Node
from Bridge import Bridge

from std_msgs.msg import String

class Node_Pub(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher = self.create_publisher(String, 'TeensyValue', 1)
        print(type(self.publisher))
        self.timerPeriod = 0.2
        self.timer = self.create_timer(self.timerPeriod, self.callback_timer)
        self.udpHelper = UDPHelper()
        self.udpHelper.open()

    def publishValue(self, value):
        msg = String(data=value)
        self.publisher.publish(msg)
        self.get_logger().info("Published value (%s)" % value)

    def callback_timer(self):
        #msg = "Hi from Bridge (" + str(self.get_clock().now()) + ")!"
        self.udpHelper.send("172.20.10.2","P","09testTopic2HeyTeensy!")
        #print("Received message \"\" from ('172.20.10.2').",sep='')
        #self.udpHelper.send("172.20.10.3","","Hello superman!")
        pass

def main(args=None):
    rclpy.init(args=args)

    nodePub = Node_Pub()
    rclpy.spin(nodePub)
    #bridge = Bridge()

    #while True:
        #bridge.Update()

    rclpy.shutdown()

    """

    global udpHelper
    udpHelper = UDPHelper()


    udpHelper.open()
    udpHelper.setCallback(nodePub.publishValue)

    rclpy.spin(nodePub)

    rclpy.shutdown()
    """

if __name__ == "__main__":
    main()