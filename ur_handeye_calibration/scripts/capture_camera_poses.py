#!/usr/bin/env python3
"""Capture camera poses for hand-eye calibration — ROS2 Humble
Use calibrator.py for full online/offline calibration."""
import sys
import signal
import numpy as np
np.set_printoptions(suppress=True, linewidth=np.inf)
import time

import rclpy
from rclpy.node import Node
import tf2_ros
from ament_index_python.packages import get_package_share_directory

from ur_control import transformations


def signal_handler(sig, frame):
    print('You pressed Ctrl+C!')
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)


class CapturePosesNode(Node):
    def __init__(self):
        super().__init__('ur_handeye_calibration_capture')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        ns = self.get_parameter_or('ur_calibration_ns', rclpy.parameter.Parameter('ur_calibration_ns', value='/')).value
        ns = ns if ns.endswith('/') else ns + '/'

        self.camera_frame = self.get_parameter_or(ns + 'tracking_base_frame', rclpy.parameter.Parameter('tracking_base_frame', value='camera_link')).value
        self.marker_frame = self.get_parameter_or(ns + 'tracking_marker_frame', rclpy.parameter.Parameter('tracking_marker_frame', value='marker_link')).value
        self.robot_frame = self.get_parameter_or(ns + 'robot_base_frame', rclpy.parameter.Parameter('robot_base_frame', value='base_link')).value
        self.endeffector_frame = self.get_parameter_or(ns + 'robot_effector_frame', rclpy.parameter.Parameter('robot_effector_frame', value='ee_link')).value

        pkg_share = get_package_share_directory('ur_handeye_calibration')
        self.savefolder = self.get_parameter_or(
            ns + 'save_to',
            rclpy.parameter.Parameter('save_to', value=pkg_share + '/config/')).value

        self.tf_data = []

    def lookup_transform(self, target, source):
        try:
            t = self.tf_buffer.lookup_transform(target, source, rclpy.time.Time())
            trans = t.transform.translation
            rot = t.transform.rotation
            return [trans.x, trans.y, trans.z, rot.x, rot.y, rot.z, rot.w]
        except Exception as e:
            self.get_logger().warn("TF lookup failed: %s" % str(e))
            return None

    def append_tf_data(self):
        time.sleep(0.1)
        obj_to_camera = self.lookup_transform(self.camera_frame, self.marker_frame)
        ee_to_base = self.lookup_transform(self.robot_frame, self.endeffector_frame)
        if obj_to_camera and ee_to_base:
            self.tf_data.append([obj_to_camera, ee_to_base])
        else:
            self.get_logger().warn("Failed to get TF data")

    def save_data(self):
        path = self.savefolder + 'calibration_data_apriltag.npy'
        np.save(path, self.tf_data)
        self.get_logger().info("Saved %d samples to %s" % (len(self.tf_data), path))


def main(args=None):
    rclpy.init(args=args)
    node = CapturePosesNode()
    rclpy.spin(node)
    node.save_data()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
