# The MIT License (MIT)
#
# Copyright (c) 2018-2023 Cristian Beltran
#
# Author: Cristian Beltran

import collections
import time
import numpy as np

import rclpy
from rclpy.duration import Duration
from geometry_msgs.msg import WrenchStamped
from std_srvs.srv import Empty, SetBool, Trigger

from ur_control import utils, spalg, conversions, transformations
from ur_control.exceptions import InverseKinematicsException
from ur_control.controllers_connection import ControllersConnection
from ur_control.controllers import JointTrajectoryController
from ur_control.grippers import GripperController, RobotiqGripper
from ur_control.constants import BASE_LINK, EE_LINK, JOINT_TRAJECTORY_CONTROLLER, FT_SUBSCRIBER, \
    ExecutionResult, IKSolverType, GripperType, get_arm_joint_names
from ur_control.ur_services import URServices

try:
    from ur_ikfast import ur_kinematics as ur_ikfast
except ImportError:
    print("Import ur_ikfast not available, IKFAST would not be supported without it")
from ur_pykdl import ur_kinematics

try:
    from trac_ik_python.trac_ik import IK as TRACK_IK_SOLVER
except ImportError:
    print("trac_ik_python not available, TRAC_IK solver not supported")
    TRACK_IK_SOLVER = None

cprint = utils.TextColors()


class Arm(object):
    """Universal Robots arm controller — ROS2 Humble"""

    def __init__(self,
                 node,
                 namespace: str = None,
                 ik_solver: IKSolverType = IKSolverType.TRAC_IK,
                 gripper_type: GripperType = GripperType.GENERIC,
                 ft_topic: str = None,
                 base_link: str = None,
                 ee_link: str = None,
                 joint_names_prefix: str = None):
        """
        Parameters
        ----------
        node : rclpy.Node
            ROS2 node instance
        namespace : optional
            ROS namespace of the robot
        ik_solver : optional
            inverse kinematic solver to be used
        gripper_type : optional
            gripper control approach
        ft_topic : optional
            topic from which to read the wrench
        base_link : optional
            robot base frame
        ee_link : optional
            end-effector frame
        joint_names_prefix : optional
            prefix for joint names when multiple robots defined
        """
        self._node = node
        self.ns = utils.solve_namespace(namespace)

        base_link = utils.resolve_parameter(value=base_link, default_value=BASE_LINK)
        ee_link = utils.resolve_parameter(value=ee_link, default_value=EE_LINK)

        self.joint_names_prefix = utils.resolve_parameter(joint_names_prefix, '')
        self.base_link = base_link if joint_names_prefix is None else joint_names_prefix + base_link
        self.ee_link = ee_link if joint_names_prefix is None else joint_names_prefix + ee_link

        self.ik_solver = ik_solver

        self.ft_topic = utils.resolve_parameter(value=ft_topic, default_value=FT_SUBSCRIBER)
        self.current_ft_value = np.zeros(6)
        self.wrench_queue = collections.deque(maxlen=25)

        self.max_joint_speed = np.deg2rad([191, 191, 191, 371, 371, 371])

        cprint.ok("Initializing ur robot with parameters")
        cprint.ok("gripper: {}, ft_sensor_topic: {}, \nbase_link: {}, ee_link: {}"
                  .format(gripper_type, self.ft_topic, self.base_link, self.ee_link))

        self.__init_controllers__(gripper_type, joint_names_prefix)
        self.__init_ik_solver__(self.base_link, self.ee_link)
        self.__init_ft_sensor__()

        self.controller_manager = ControllersConnection(self._node, self.ns)
        self.dashboard_services = URServices(self._node, self.ns)

    def __init_controllers__(self, gripper_type, joint_names_prefix=None):
        self.joint_names = None if joint_names_prefix is None else get_arm_joint_names(joint_names_prefix)

        self.joint_traj_controller = JointTrajectoryController(
            node=self._node,
            publisher_name=JOINT_TRAJECTORY_CONTROLLER,
            namespace=self.ns,
            joint_names=self.joint_names,
            timeout=1.0)

        self.gripper = None

        if not gripper_type:
            self._node.get_logger().warn("Loading without gripper")
            return

        if gripper_type == GripperType.GENERIC:
            self.gripper = GripperController(node=self._node, namespace=self.ns, prefix=self.joint_names_prefix, timeout=2.0)
        elif gripper_type == GripperType.ROBOTIQ:
            self.gripper = RobotiqGripper(node=self._node, namespace=self.ns, prefix=self.joint_names_prefix, timeout=2.0)
        else:
            raise ValueError("Invalid gripper type %s" % gripper_type)

    def __init_ik_solver__(self, base_link, ee_link):
        if self._node.has_parameter("robot_description"):
            self.kdl = ur_kinematics(base_link=base_link, ee_link=ee_link)
        else:
            raise ValueError("robot_description not found in the parameter server")

        if self.ik_solver == IKSolverType.IKFAST:
            try:
                self.arm_ikfast = ur_ikfast.URKinematics(self._robot_urdf)
            except Exception:
                raise ValueError("IK solver set to IKFAST but no ikfast found for: %s. " % self._robot_urdf)
        elif self.ik_solver == IKSolverType.TRAC_IK:
            if TRACK_IK_SOLVER is None:
                raise RuntimeError("trac_ik_python not available")
            try:
                self.trac_ik = TRACK_IK_SOLVER(base_link=base_link, tip_link=ee_link, solve_type="Distance")
            except Exception as e:
                self._node.get_logger().error("Could not instantiate TRAC_IK: " + str(e))
        elif self.ik_solver == IKSolverType.KDL:
            pass
        else:
            raise Exception("unsupported ik_solver", self.ik_solver)

    def __init_ft_sensor__(self):
        ft_namespace = self.ns.rstrip('/') + '/' + self.ft_topic + '/filtered'
        if not utils.topic_exist(self._node, ft_namespace):
            self._node.get_logger().warn("Filtered FT topic not found. Using raw sensor directly.")
            ft_namespace = self.ns.rstrip('/') + '/' + self.ft_topic
            self._node.create_subscription(WrenchStamped, ft_namespace, self.__ft_callback__, 1)
            self._zero_ft_filtered = lambda: None
            self._ft_filtered = lambda _: None
        else:
            self._node.create_subscription(WrenchStamped, ft_namespace, self.__ft_callback__, 1)

            self._zero_ft_filtered_client = self._node.create_client(
                Empty, '%s/%s/filtered/zero_ftsensor' % (self.ns.rstrip('/'), self.ft_topic))
            self._zero_ft_filtered_client.wait_for_service(timeout_sec=2.0)

            if not self._node.has_parameter("use_gazebo_sim"):
                self._zero_ft_client = self._node.create_client(
                    Trigger, '%s/ur_hardware_interface/zero_ftsensor' % self.ns.rstrip('/'))
                self._zero_ft_client.wait_for_service(timeout_sec=2.0)

            self._ft_filtered_client = self._node.create_client(
                SetBool, '%s/%s/filtered/enable_filtering' % (self.ns.rstrip('/'), self.ft_topic))
            self._ft_filtered_client.wait_for_service(timeout_sec=1.0)

            if not utils.wait_for(lambda: self.current_ft_value is not None, timeout=2.0):
                self._node.get_logger().error('Timed out waiting for {0} topic'.format(ft_namespace))

    def __ft_callback__(self, msg):
        self.current_ft_value = conversions.from_wrench(msg.wrench)
        self.wrench_queue.append(self.current_ft_value)

    def _call_service(self, client, request, timeout=5.0):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=timeout)
        return future.result()

### Data access methods ###

    def inverse_kinematics(self, pose, seed=None, attempts=0, verbose=True):
        q_guess_ = seed if seed is not None else self.joint_angles()

        if self.ik_solver == IKSolverType.IKFAST:
            ik = self.arm_ikfast.inverse(pose, q_guess=q_guess_)
        elif self.ik_solver == IKSolverType.TRAC_IK:
            ik = self.trac_ik.get_ik(q_guess_, *pose)
        elif self.ik_solver == IKSolverType.KDL:
            ik = self.kdl.inverse_kinematics(pose[:3], pose[3:], seed=q_guess_)

        if ik is None:
            if attempts > 0:
                return self.inverse_kinematics(pose, seed, attempts - 1)
            if verbose:
                self._node.get_logger().warn(f"{self.ik_solver}: solution not found!")
            raise InverseKinematicsException(f"{self.ik_solver}: solution not found!")
        return ik

    def end_effector(self, joint_angles=None, rot_type='quaternion', tip_link=None):
        joint_angles = self.joint_angles() if joint_angles is None else joint_angles

        if rot_type == 'quaternion':
            return self.kdl.forward(joint_angles, tip_link)
        elif rot_type == 'euler':
            x = self.end_effector(joint_angles, tip_link=tip_link)
            euler = np.array(transformations.euler_from_quaternion(x[3:], axes='sxyz'))
            return np.concatenate((x[:3], euler))
        elif rot_type == 'ortho6':
            x = self.end_effector(joint_angles, tip_link=tip_link)
            ortho6 = np.array(transformations.ortho6_from_quaternion(x[3:]))
            return np.concatenate((x[:3], ortho6))
        else:
            raise ValueError("Rotation Type not supported", rot_type)

    def joint_angle(self, joint):
        joint_idx = self.joint_traj_controller.valid_joint_names.index(joint)
        return self.joint_traj_controller.get_joint_positions()[joint_idx]

    def joint_angles(self):
        return self.joint_traj_controller.get_joint_positions()

    def joint_velocity(self, joint):
        joint_idx = self.joint_traj_controller.valid_joint_names.index(joint)
        return self.joint_traj_controller.get_joint_velocities()[joint_idx]

    def joint_velocities(self):
        return self.joint_traj_controller.get_joint_velocities()

    def joint_effort(self, joint):
        joint_idx = self.joint_traj_controller.valid_joint_names.index(joint)
        return self.joint_traj_controller.get_joint_efforts()[joint_idx]

    def joint_efforts(self):
        return self.joint_traj_controller.get_joint_efforts()

    def get_wrench_history(self, hist_size=24, hand_frame_control=False):
        if self.current_ft_value is None:
            raise Exception("FT Sensor not initialized")

        ft_hist = np.array(self.wrench_queue)[:hist_size]

        if hand_frame_control:
            q_hist = self.joint_traj_controller.get_joint_positions_hist()[:hist_size]
            poses_hist = [self.end_effector(q, tip_link=self.ee_link) for q in q_hist]
            wrench_hist = [spalg.convert_wrench(wft, p).tolist() for p, wft in zip(poses_hist, ft_hist)]
        else:
            wrench_hist = ft_hist

        return np.array(wrench_hist)

    def get_wrench(self, base_frame_control=False, hand_frame_control=False):
        if self.current_ft_value is None:
            return np.zeros(6)

        wrench_force = self.current_ft_value
        if not hand_frame_control and not base_frame_control:
            return wrench_force

        if base_frame_control:
            transform = self.end_effector(tip_link=self.joint_names_prefix + "wrist_3_link")
            return spalg.convert_wrench(wrench_force, transform)
        else:
            transform = self.end_effector(tip_link=self.ee_link)
            return spalg.convert_wrench(wrench_force, transform)

### Control Methods ###

    def set_joint_positions(self, target_time, positions, velocities=None, accelerations=None, wait=False):
        self.joint_traj_controller.add_point(
            positions=positions,
            velocities=velocities,
            accelerations=accelerations,
            target_time=target_time)
        if wait:
            self.joint_traj_controller.start(delay=0, wait=True)
        else:
            self.joint_traj_controller.start_no_action_server()

        self.joint_traj_controller.clear_points()

        if wait:
            res = self.joint_traj_controller.get_result()
            return ExecutionResult.DONE if (res and res.error_code == 0) else ExecutionResult.CONTROLLER_FAILED
        return ExecutionResult.DONE

    def set_joint_trajectory(self, target_time, trajectory, velocities=None, accelerations=None):
        dt = target_time / len(trajectory)
        for i, q in enumerate(trajectory):
            self.joint_traj_controller.add_point(
                positions=q,
                target_time=(i + 1) * dt,
                velocities=velocities,
                accelerations=accelerations)
        self.joint_traj_controller.start(delay=0, wait=True)
        self.joint_traj_controller.clear_points()

        res = self.joint_traj_controller.get_result()
        return ExecutionResult.DONE if (res and res.error_code == 0) else ExecutionResult.CONTROLLER_FAILED

    def set_target_pose(self, target_time, pose, wait=False):
        q = self.inverse_kinematics(pose)
        if q is None:
            self._node.get_logger().debug("IK not found")
            raise InverseKinematicsException("IK solver failed to find a solution")
        return self.set_joint_positions(positions=q, target_time=target_time, wait=wait)

    def set_pose_trajectory(self, target_time, trajectory):
        joint_trajectory = []
        previous_q = self.joint_angles()
        for pose in trajectory:
            q = self.inverse_kinematics(pose, seed=previous_q)
            if q is None:
                raise InverseKinematicsException("IK solver failed to find a solution")
            previous_q = q
            joint_trajectory.append(q)
        return self.set_joint_trajectory(trajectory=joint_trajectory, target_time=target_time)

    def move_relative(self, target_time, transformation, relative_to_tcp=True, wait=True):
        new_pose = transformations.transform_pose(self.end_effector(), transformation, rotated_frame=relative_to_tcp)
        return self.set_target_pose(pose=new_pose, target_time=target_time, wait=wait)

### FT sensor control ###

    def zero_ft_sensor(self):
        if not self._node.has_parameter("use_gazebo_sim"):
            self._call_service(self._zero_ft_client, Trigger.Request())
            time.sleep(0.5)
        self._call_service(self._zero_ft_filtered_client, Empty.Request())

    def set_ft_filtering(self, active=True):
        req = SetBool.Request()
        req.data = active
        self._call_service(self._ft_filtered_client, req)
