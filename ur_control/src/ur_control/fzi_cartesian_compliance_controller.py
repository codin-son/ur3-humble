# The MIT License (MIT)
# Copyright (c) 2023 Cristian Beltran
# Author: Cristian Beltran

import collections
import threading
import types
import rclpy
import numpy as np
import time

from ur_control.arm import Arm
from ur_control import conversions
from ur_control.constants import JOINT_TRAJECTORY_CONTROLLER, CARTESIAN_COMPLIANCE_CONTROLLER, ExecutionResult

from geometry_msgs.msg import WrenchStamped, PoseStamped
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType


def is_more_extreme(value, target):
    if (np.all(value > 0) and np.all(target > 0)):
        return np.all(value > target)
    elif (np.all(value < 0) and np.all(target < 0)):
        return np.all(value < target)
    return False


def convert_selection_matrix_to_parameters(selection_matrix):
    return {
        "stiffness": {
            "sel_x": selection_matrix[0],
            "sel_y": selection_matrix[1],
            "sel_z": selection_matrix[2],
            "sel_ax": selection_matrix[3],
            "sel_ay": selection_matrix[4],
            "sel_az": selection_matrix[5],
        }
    }


def convert_stiffness_to_parameters(stiffness):
    return {
        "stiffness": {
            "trans_x": stiffness[0],
            "trans_y": stiffness[1],
            "trans_z": stiffness[2],
            "rot_x": stiffness[3],
            "rot_y": stiffness[4],
            "rot_z": stiffness[5],
        }
    }


def convert_pd_gains_to_parameters(p_gains, d_gains=[0, 0, 0, 0, 0, 0]):
    return {
        "trans_x": {"p": p_gains[0], "d": d_gains[0]},
        "trans_y": {"p": p_gains[1], "d": d_gains[1]},
        "trans_z": {"p": p_gains[2], "d": d_gains[2]},
        "rot_x": {"p": p_gains[3], "d": d_gains[3]},
        "rot_y": {"p": p_gains[4], "d": d_gains[4]},
        "rot_z": {"p": p_gains[5], "d": d_gains[5]}
    }


def switch_cartesian_controllers(func):
    """Decorator: switch from cartesian to joint trajectory controllers and back."""
    def wrap(*args, **kwargs):
        if not args[0].auto_switch_controllers:
            return func(*args, **kwargs)
        args[0].activate_cartesian_controller()
        try:
            res = func(*args, **kwargs)
        except Exception as e:
            args[0]._node.get_logger().error("Exception: %s" % e)
            res = ExecutionResult.DONE
        args[0].activate_joint_trajectory_controller()
        return res
    return wrap


class CompliantController(Arm):
    """Compliant controller using FZI Cartesian Compliance controllers — ROS2 Humble"""

    def __init__(self, **kwargs):
        Arm.__init__(self, **kwargs)

        self.is_gazebo_sim = self._node.has_parameter("use_gazebo_sim")
        self.min_dt = 1.0 / self.joint_traj_controller.rate if hasattr(self.joint_traj_controller, 'rate') else 0.002
        self.rate_period = self.min_dt

        self.auto_switch_controllers = True
        self.current_target_pose = np.zeros(7)
        self.current_wrench_pose = np.zeros(6)

        ns = self.ns.rstrip('/')
        cc = CARTESIAN_COMPLIANCE_CONTROLLER

        self._node.create_subscription(
            PoseStamped, '%s/%s/target_frame' % (ns, cc), self.target_pose_cb, 10)
        self._node.create_subscription(
            WrenchStamped, '%s/%s/target_wrench' % (ns, cc), self.target_wrench_cb, 10)

        self.cartesian_target_pose_pub = self._node.create_publisher(
            PoseStamped, '%s/%s/target_frame' % (ns, cc), 10)
        self.cartesian_target_wrench_pub = self._node.create_publisher(
            WrenchStamped, '%s/%s/target_wrench' % (ns, cc), 10)

        # ROS2: Use set_parameters service to update FZI controller params
        # The FZI cartesian compliance controller exposes parameters via ROS2 parameter interface
        self._param_clients = {}
        for param_ns in ['pd_gains/trans_x', 'pd_gains/trans_y', 'pd_gains/trans_z',
                          'pd_gains/rot_x', 'pd_gains/rot_y', 'pd_gains/rot_z',
                          'stiffness', 'force', 'solver', '']:
            node_name = '%s/%s/%s' % (ns, cc, param_ns) if param_ns else '%s/%s' % (ns, cc)
            key = param_ns.split('/')[-1] if param_ns else 'end_effector_link'
            self._param_clients[key] = self._node.create_client(
                SetParameters, node_name.lstrip('/') + '/set_parameters')

        self.param_update_queue = collections.deque(maxlen=15)
        self.update_thread = None
        self.update_lock = threading.Lock()
        self.update_condition = threading.Condition()
        self.update_thread_stopped = False
        self.async_mode = False

        self.set_hand_frame_control(False)
        self.set_end_effector_link(self.ee_link)
        self.min_scale_error = 1.5

    def __del__(self):
        if hasattr(self, 'update_condition'):
            with self.update_condition:
                self.update_thread_stopped = True
                self.update_condition.notify_all()

    def target_pose_cb(self, data):
        self.current_target_pose = conversions.from_pose_to_list(data.pose)

    def target_wrench_cb(self, data):
        self.current_target_wrench = conversions.from_wrench(data.wrench)

    def activate_cartesian_controller(self):
        return self.controller_manager.switch_controllers(
            controllers_on=[CARTESIAN_COMPLIANCE_CONTROLLER],
            controllers_off=[JOINT_TRAJECTORY_CONTROLLER])

    def activate_joint_trajectory_controller(self):
        return self.controller_manager.switch_controllers(
            controllers_on=[JOINT_TRAJECTORY_CONTROLLER],
            controllers_off=[CARTESIAN_COMPLIANCE_CONTROLLER])

    def set_cartesian_target_wrench(self, wrench: list):
        try:
            target_wrench = WrenchStamped()
            target_wrench.header.frame_id = self.base_link
            target_wrench.wrench = conversions.to_wrench(wrench)
            self.cartesian_target_wrench_pub.publish(target_wrench)
        except Exception as e:
            self._node.get_logger().error("Fail to set_target_wrench(): %s" % e)

    def set_cartesian_target_pose(self, pose: list):
        try:
            target_pose = conversions.to_pose_stamped(self.base_link, pose)
            self.cartesian_target_pose_pub.publish(target_pose)
        except Exception as e:
            self._node.get_logger().error("Fail to set_target_pose(): %s" % e)

    def _set_ros2_parameters(self, client_key, params_dict):
        """Set parameters on FZI controller node via ROS2 set_parameters service."""
        client = self._param_clients.get(client_key)
        if client is None or not client.service_is_ready():
            return
        req = SetParameters.Request()
        for name, value in params_dict.items():
            param = Parameter()
            param.name = name
            if isinstance(value, bool):
                param.value = ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=value)
            elif isinstance(value, int):
                param.value = ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=value)
            elif isinstance(value, float):
                param.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=value)
            elif isinstance(value, str):
                param.value = ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=value)
            req.parameters.append(param)
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=1.0)

    def publish_parameter_update(self, parameters):
        try:
            for param_group, param_dict in parameters.items():
                self._set_ros2_parameters(param_group, param_dict)
        except Exception as e:
            self._node.get_logger().error("failed publish_parameter_update %s" % e)

    def __update_controller_parameter_loop__(self):
        while rclpy.ok():
            if self.update_thread_stopped:
                return
            if not self.param_update_queue:
                with self.update_condition:
                    self.update_condition.wait(timeout=0.5)
                return
            else:
                with self.update_lock:
                    parameters = self.param_update_queue.pop()
                if parameters:
                    self.publish_parameter_update(parameters)
                    parameters = None

    def update_controller_parameters(self, parameters: dict):
        if self.async_mode:
            with self.update_lock:
                self.param_update_queue.append(parameters)
            if self.update_thread is None or not self.update_thread.is_alive():
                del self.update_thread
                self.update_thread = threading.Thread(target=self.__update_controller_parameter_loop__)
                self.update_thread.start()
            with self.update_condition:
                self.update_condition.notify()
        else:
            self.publish_parameter_update(parameters)

    def update_selection_matrix(self, selection_matrix):
        parameters = convert_selection_matrix_to_parameters(selection_matrix)
        self.update_controller_parameters(parameters)

    def update_pd_gains(self, p_gains, d_gains=[0, 0, 0, 0, 0, 0]):
        parameters = convert_pd_gains_to_parameters(p_gains, d_gains)
        self.update_controller_parameters(parameters)

    def update_stiffness(self, stiffness):
        parameters = convert_stiffness_to_parameters(stiffness)
        self.update_controller_parameters(parameters)

    def set_control_mode(self, mode="parallel"):
        parameters = {"stiffness": {}}
        if mode == "parallel":
            parameters["stiffness"].update({"use_parallel_force_position_control": True})
        elif mode == "spring-mass-damper":
            parameters["stiffness"].update({"use_parallel_force_position_control": False})
        else:
            raise ValueError("Unknown control mode %s" % mode)
        self.update_controller_parameters(parameters)

    def set_position_control_mode(self, enable=True):
        parameters = convert_selection_matrix_to_parameters(np.ones(6))
        parameters["stiffness"].update({"use_parallel_force_position_control": enable})
        self.update_controller_parameters(parameters)

    def set_hand_frame_control(self, enable):
        parameters = {"hand_frame_control": {"hand_frame_control": enable}}
        self.update_controller_parameters(parameters)

    def set_end_effector_link(self, end_effector_link):
        parameters = {"end_effector_link": {"end_effector_link": end_effector_link}}
        self.update_controller_parameters(parameters)

    def set_solver_parameters(self, error_scale=None, iterations=None, publish_state_feedback=None):
        parameters = {"solver": {}}
        if error_scale:
            error_scale = error_scale if not self.is_gazebo_sim else error_scale * 0.01
            parameters["solver"].update({"error_scale": round(error_scale, 4)})
        if iterations:
            parameters["solver"].update({"iterations": iterations})
        if publish_state_feedback:
            parameters["solver"].update({"publish_state_feedback": publish_state_feedback})
        self.update_controller_parameters(parameters)

    def wait_for_robot_to_stop(self, wait_time=5):
        remaining_time = wait_time
        start_time = time.time()
        prev_state = self.joint_angles()
        no_motion_count = 0
        while remaining_time > 0 and no_motion_count < 3:
            time.sleep(self.rate_period)
            remaining_time = wait_time - (time.time() - start_time)
            curr_state = self.joint_angles()
            if np.allclose(prev_state, curr_state, atol=0.0001):
                no_motion_count += 1
            else:
                no_motion_count = 0

    @switch_cartesian_controllers
    def execute_compliance_control(self, trajectory: np.array, target_wrench: np.array, max_force_torque: list,
                                   duration: float, stop_on_target_force=False, termination_criteria=None,
                                   auto_stop=True, func=None, scale_up_error=False, max_scale_error=None,
                                   relative_to_ee=False, stop_at_wrench=None):

        trajectory = trajectory.reshape((-1, 7))
        step_duration = max(self.min_dt, duration / float(trajectory.shape[0]))
        trajectory_index = 0

        initial_time = time.time()
        step_initial_time = time.time()

        result = ExecutionResult.DONE
        if stop_on_target_force and stop_at_wrench is None:
            raise ValueError("'stop_at_wrench' not specified when requesting 'stop_on_target_force'")

        if stop_on_target_force:
            stop_at_wrench = np.array(stop_at_wrench)
            stop_target_wrench_mask = np.flatnonzero(stop_at_wrench)

        self.set_cartesian_target_wrench(target_wrench)
        self.set_cartesian_target_pose(trajectory[trajectory_index])

        if scale_up_error and max_scale_error:
            self.sliding_error(trajectory[trajectory_index], max_scale_error)

        while rclpy.ok() and (time.time() - initial_time) < duration:
            current_wrench = self.get_wrench(base_frame_control=True)

            if termination_criteria is not None:
                assert isinstance(termination_criteria, types.LambdaType)
                if termination_criteria(self.end_effector()):
                    self._node.get_logger().info("Termination criteria returned True, stopping force control")
                    result = ExecutionResult.TERMINATION_CRITERIA
                    break

            if stop_on_target_force and is_more_extreme(current_wrench[stop_target_wrench_mask], stop_at_wrench[stop_target_wrench_mask]):
                self._node.get_logger().info('Target F/T reached {} Stopping!'.format(np.round(current_wrench, 2)))
                result = ExecutionResult.STOP_ON_TARGET_FORCE
                break

            if np.any(np.abs(current_wrench) > max_force_torque):
                self._node.get_logger().error('Maximum force/torque exceeded {}'.format(np.round(current_wrench, 3)))
                result = ExecutionResult.FORCE_TORQUE_EXCEEDED
                break

            if (time.time() - step_initial_time) > step_duration:
                step_initial_time = time.time()
                trajectory_index += 1
                if trajectory_index >= trajectory.shape[0]:
                    break
                self.set_cartesian_target_pose(trajectory[trajectory_index])
                if scale_up_error and max_scale_error:
                    self.sliding_error(trajectory[trajectory_index], max_scale_error)

            if func:
                func(self.end_effector())

            time.sleep(self.rate_period)

        if auto_stop:
            self.set_cartesian_target_pose(self.end_effector())
            self.set_position_control_mode()
            self.wait_for_robot_to_stop(wait_time=5)

        return result

    def sliding_error(self, target_pose, max_scale_error):
        position_error = np.linalg.norm(target_pose[:3] - self.end_effector()[:3])
        factor = 1 - np.tanh(100 * position_error)
        scale_error = np.interp(factor, [0, 1], [self.min_scale_error, max_scale_error])
        self.set_solver_parameters(error_scale=np.round(scale_error, 3))
