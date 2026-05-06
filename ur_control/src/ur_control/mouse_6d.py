import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
import time


class Mouse6D():
    """Subscribe to 3DConnexion mouse, convert messages — ROS2 Humble"""
    def __init__(self, node):
        self._node = node
        self._node.create_subscription(Twist, 'spacenav/twist', self.twist_cb, 1)
        self._node.create_subscription(Joy, 'spacenav/joy', self.joy_cb, 1)
        self.twist = None
        self.joy_axes = None
        self.joy_buttons = None
        time.sleep(0.01)

    def twist_cb(self, msg):
        self.twist = [
            msg.linear.x, msg.linear.y, msg.linear.z,
            msg.angular.x, msg.angular.y, msg.angular.z,
        ]

    def joy_cb(self, msg):
        self.joy_axes = msg.axes
        self.joy_buttons = msg.buttons


def main(args=None):
    rclpy.init(args=args)
    node = Node("Mouse6D")
    Mouse6D(node)
    time.sleep(1)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
