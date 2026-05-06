#!/usr/bin/env python3
"""Hand-eye calibration capture/compute — ROS2 Humble"""
import sys
import signal
import numpy as np
np.set_printoptions(suppress=True, linewidth=np.inf)

import rclpy
from rclpy.node import Node
import tf2_ros
from ament_index_python.packages import get_package_share_directory

from ur_control import transformations, conversions

try:
    from handeye.srv import CalibrateHandEye
except ImportError:
    print("handeye service not available")
    CalibrateHandEye = None


def signal_handler(sig, frame):
    print('You pressed Ctrl+C!')
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)


class HandEyeCalibrator(Node):
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

        self.camera_setup = self.get_parameter_or(ns + 'camera_setup', rclpy.parameter.Parameter('camera_setup', value='Fixed')).value
        self.calibration_solver = self.get_parameter_or(ns + 'solver', rclpy.parameter.Parameter('solver', value='Daniilidis1999')).value

        pkg_share = get_package_share_directory('ur_handeye_calibration')
        self.savefolder = self.get_parameter_or(
            ns + 'calibration_file',
            rclpy.parameter.Parameter('calibration_file', value=pkg_share + '/config/')).value

        self.tf_data = []
        self.arm = None
        self.ns = ns

    def lookup_transform(self, target, source):
        """Lookup TF2 transform, return [x,y,z,qx,qy,qz,qw]."""
        import time
        try:
            t = self.tf_buffer.lookup_transform(target, source, rclpy.time.Time())
            trans = t.transform.translation
            rot = t.transform.rotation
            return [trans.x, trans.y, trans.z, rot.x, rot.y, rot.z, rot.w]
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn("TF lookup failed: %s" % str(e))
            return None

    def append_tf_data(self):
        import time
        time.sleep(0.1)
        obj_to_camera = self.lookup_transform(self.camera_frame, self.marker_frame)
        ee_to_base = self.lookup_transform(self.robot_frame, self.endeffector_frame)
        if obj_to_camera and ee_to_base:
            self.tf_data.append([obj_to_camera, ee_to_base])
        else:
            self.get_logger().warn("Failed to get TF data")

    def run_calibration(self):
        if CalibrateHandEye is None:
            self.get_logger().error("handeye service not available")
            return

        online = self.get_parameter_or(
            self.ns + 'online',
            rclpy.parameter.Parameter('online', value=False)).value

        if online:
            from ur_control.compliance_controller import ComplianceController
            self.arm = ComplianceController(node=self, ft_sensor=False)
            self._move_arm()
            np.save(self.savefolder + "calibration_data_apriltag.npy", self.tf_data)
        else:
            self.tf_data = np.load(self.savefolder + "calibration_data_apriltag.npy")

        srv_client = self.create_client(CalibrateHandEye, 'handeye_calibration')
        if not srv_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("handeye_calibration service not available")
            return

        req = CalibrateHandEye.Request()
        req.setup = self.camera_setup
        req.solver = self.calibration_solver
        bte = []
        cto = []
        for data in self.tf_data:
            cto.append(conversions.to_pose_msg(data[0]))
            bte.append(conversions.to_pose_msg(data[1]))
        req.effector_wrt_world.poses = bte
        req.object_wrt_sensor.poses = cto

        future = srv_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        print("success", response.success)
        print("rotation_rmse", response.rotation_rmse)
        print("translation_rmse", response.translation_rmse)
        print("sensor_frame", response.sensor_frame)

    def _move_arm(self):
        import time
        q = [2.37191, -1.88688, -1.82035, 0.4766, 2.31206, 3.18758]
        self.arm.set_joint_positions(positions=q, wait=True, target_time=0.5)
        initial_ee = self.arm.end_effector(q)

        deltas = [
            [0.0, 0.03, 0.08, 0.13, 0.18],
            [0.0, 0.03, 0.08, 0.13, 0.18],
            [0.0, 0.03, 0.08, 0.13, 0.18],
        ]

        pose_changes = [
            [[4, 2.05], [5, 3.20]], [[4, 2.3], [5, 3.20]], [[4, 2.1936], [5, 3.8]],
            [[4, 2.05], [5, 3.8]], [[4, 2.3], [5, 3.8]], [[4, 2.1936], [5, 2.6]],
            [[4, 2.05], [5, 2.6]], [[4, 2.3], [5, 2.6]],
        ]

        X = Y = 5
        Z = 3
        for i in range(Z):
            x = y = 0
            dx, dy = 0, -1
            for _ in range(max(X, Y)**2):
                if (-X/2 < x <= X/2) and (-Y/2 < y <= Y/2):
                    delta = np.zeros(6)
                    delta[0] = deltas[0][x+1]
                    delta[2] = deltas[2][y+1]
                    delta[1] = deltas[1][i]
                    cpose = transformations.pose_euler_to_quaternion(initial_ee, delta, ee_rotation=False)
                    self.arm.set_target_pose(pose=cpose, wait=True, target_time=0.5)
                    self.append_tf_data()
                    cq = np.copy(self.arm.joint_angles())
                    for change in pose_changes:
                        for p in change:
                            cq[p[0]] = p[1]
                        self.arm.set_joint_positions(cq, wait=True, target_time=0.5)
                        self.append_tf_data()
                if x == y or (x < 0 and x == -y) or (x > 0 and x == 1-y):
                    dx, dy = -dy, dx
                x, y = x+dx, y+dy


def main(args=None):
    rclpy.init(args=args)
    node = HandEyeCalibrator()
    node.run_calibration()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
