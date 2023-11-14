import rclpy
from Bridge import Bridge
import traceback
import time
import os

current_pid = os.getpid()

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