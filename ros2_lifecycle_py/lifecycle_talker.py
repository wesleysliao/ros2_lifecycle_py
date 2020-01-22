
import time

import rclpy
from std_msgs.msg import String

from lifecycle_msgs.msg import State
from lifecycle_msgs.msg import Transition

from ros2_lifecycle_py.lifecycle import LifecycleNode

class LifecycleTalker (LifecycleNode):
    def __init__(self):
        super().__init__("lc_talker")
    
    def on_configure(self):
        self.pub = self.create_publisher(String, "lifecycle_chatter", 10)
        self.get_logger().info("on_configure() is called")

        return Transition.TRANSITION_CALLBACK_SUCCESS
    
    def on_activate(self):
        self.get_logger().info("on_activate() is called")
        time.sleep(2)

        return Transition.TRANSITION_CALLBACK_SUCCESS
    
    def on_deactivate(self):
        self.get_logger().info("on_deactivate() is called")

        return Transition.TRANSITION_CALLBACK_SUCCESS


def main(args=None):
    rclpy.init(args=args)

    lifecycle_talker = LifecycleTalker()
    
    rclpy.spin(lifecycle_talker)

    rclpy.shutdown()


if __name__ == "__main__":
    main()