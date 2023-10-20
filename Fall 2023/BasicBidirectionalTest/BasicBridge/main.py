from UDPHelper import UDPHelper
import rclpy
from rclpy.node import Node
from Bridge import Bridge
import traceback
import time

from std_msgs.msg import String

class Node_Pub(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher = self.create_publisher(String, 'TeensyValue', 1)
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
    bridge = Bridge()

    error_exception = None
    error_traceback = None

    try:
        while rclpy.ok():
            bridge.Update()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        error_exception = e
        error_traceback = traceback.format_exc()

    bridge.close()
    rclpy.shutdown()

    if error_exception is not None:
        time.sleep(1)
        print()
        print()
        print("========== ERROR CAUGHT ==========")
        print()
        print("Exception:", error_exception)
        print(error_traceback)

if __name__ == "__main__":
    main()