#!/usr/bin/env python3
"""Offline hand-eye calibration using saved .npy data — ROS2 Humble"""
import sys
import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import TransformStamped
import time

try:
    from handeye import HandEyeCalibrator, Setup, solver
except ImportError:
    HandEyeCalibrator = None

import numpy as np
np.set_printoptions(suppress=True, linewidth=np.inf)

try:
    import baldor as br
except ImportError:
    br = None

from pyquaternion import Quaternion
from ur_control import transformations as tr
from ament_index_python.packages import get_package_share_directory


class CalibrateNode(Node):
    def __init__(self):
        super().__init__('ur3_force_control')
        pkg_share = get_package_share_directory('ur_handeye_calibration')
        self.calibration_file = self.get_parameter_or(
            'calibration_file',
            rclpy.parameter.Parameter('calibration_file',
                                      value=pkg_share + '/config/calibration_data_apriltag.npy')).value
        self.br = tf2_ros.TransformBroadcaster(self)

    def calibrate_simulation(self, cto_poses, bte_poses):
        if HandEyeCalibrator is None:
            self.get_logger().error("handeye package not available")
            return
        calibrator = HandEyeCalibrator(setup=Setup.Fixed)
        for cto_pose, bte_pose in zip(cto_poses, bte_poses):
            bte = tr.pose_to_transform(bte_pose)
            cto = tr.pose_to_transform(cto_pose)
            calibrator.assess_tcp_pose(bte)
            calibrator.add_sample(bte, cto)

        Xest = calibrator.solve(method=solver.Daniilidis1999)
        Xpose = tr.pose_quaternion_from_matrix(Xest)
        print("via Daniilidis1999:", Xpose)

        self.timer = self.create_timer(0.1, lambda: self._publish_tf(Xpose))

    def _publish_tf(self, Xpose):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "base_link"
        t.child_frame_id = "camera_es_link"
        t.transform.translation.x = Xpose[0]
        t.transform.translation.y = Xpose[1]
        t.transform.translation.z = Xpose[2]
        t.transform.rotation.x = Xpose[3]
        t.transform.rotation.y = Xpose[4]
        t.transform.rotation.z = Xpose[5]
        t.transform.rotation.w = Xpose[6]
        self.br.sendTransform(t)

    def run(self):
        all_poses = np.load(self.calibration_file)
        cto_poses = all_poses[:, 0, :]
        bte_poses = all_poses[:, 1, :]
        self.calibrate_simulation(cto_poses, bte_poses)


def main(args=None):
    rclpy.init(args=args)
    node = CalibrateNode()
    node.run()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
