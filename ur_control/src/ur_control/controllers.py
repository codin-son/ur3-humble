#!/usr/bin/env python3
import copy
import collections
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from ur_control import constants
import numpy as np
from std_msgs.msg import Float64
from controller_manager_msgs.srv import ListControllers
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory


class JointControllerBase(object):
    """
    Base class for Joint Position Controllers. Subscribes to joint_states topic.
    Requires a rclpy.Node instance passed as `node`.
    """

    def __init__(self, node, namespace, timeout, joint_names=None):
        self._node = node
        self.valid_joint_names = constants.JOINT_ORDER if joint_names is None else joint_names
        self.ns = namespace if namespace else '/'
        self._jnt_positions_hist = collections.deque(maxlen=24)
        self._joint_names = None
        self._current_jnt_positions = None
        self._current_jnt_velocities = None
        self._current_jnt_efforts = None

        self._js_sub = self._node.create_subscription(
            JointState, 'joint_states', self.joint_states_cb, 1)

        start_time = self._node.get_clock().now()
        retry = False
        while self._joint_names is None:
            elapsed = (self._node.get_clock().now() - start_time).nanoseconds / 1e9
            if elapsed > timeout and not retry:
                # retry with namespace
                topic = (self.ns.rstrip('/') + '/joint_states').lstrip('/')
                self._js_sub = self._node.create_subscription(
                    JointState, topic, self.joint_states_cb, 1)
                start_time = self._node.get_clock().now()
                retry = True
                continue
            elif elapsed > timeout and retry:
                self._node.get_logger().error('Timed out waiting for joint_states topic')
                return
            rclpy.spin_once(self._node, timeout_sec=0.01)

        self._num_joints = len(self._joint_names)
        self._node.get_logger().debug('Topic [joint_states] found')

    def disconnect(self):
        self._node.destroy_subscription(self._js_sub)

    def get_joint_efforts(self):
        return np.array(self._current_jnt_efforts)

    def get_joint_positions(self):
        return np.array(self._current_jnt_positions)

    def get_joint_positions_hist(self):
        return list(self._jnt_positions_hist)

    def get_joint_velocities(self):
        return np.array(self._current_jnt_velocities)

    def joint_states_cb(self, msg):
        position = []
        velocity = []
        effort = []
        name = []
        for joint_name in self.valid_joint_names:
            if joint_name in msg.name:
                idx = msg.name.index(joint_name)
                name.append(msg.name[idx])
                if msg.effort:
                    effort.append(msg.effort[idx])
                if msg.velocity:
                    velocity.append(msg.velocity[idx])
                position.append(msg.position[idx])
        if set(name) == set(self.valid_joint_names):
            self._current_jnt_positions = np.array(position)
            self._jnt_positions_hist.append(self._current_jnt_positions)
            self._current_jnt_velocities = np.array(velocity)
            self._current_jnt_efforts = np.array(effort)
            self._joint_names = list(name)


class JointPositionController(JointControllerBase):
    """Joint Position Controller — publishes Float64 per joint."""

    def __init__(self, node, namespace='', timeout=5.0, joint_names=None):
        super().__init__(node, namespace, timeout=timeout, joint_names=joint_names)
        if self._joint_names is None:
            raise RuntimeError('JointPositionController timed out waiting joint_states: %s' % namespace)
        self._cmd_pub = {}
        ns_prefix = (self.ns.rstrip('/') + '/') if self.ns != '/' else '/'
        for joint in self._joint_names:
            topic = '%s%scommand' % (ns_prefix, joint)
            self._cmd_pub[joint] = self._node.create_publisher(Float64, topic, 3)

        # Wait for controller_manager list_controllers
        srv_name = ns_prefix + 'controller_manager/list_controllers'
        list_client = self._node.create_client(ListControllers, srv_name)
        list_client.wait_for_service(timeout_sec=timeout)
        expected = joint_names if joint_names is not None else constants.JOINT_ORDER
        start_time = self._node.get_clock().now()
        while True:
            elapsed = (self._node.get_clock().now() - start_time).nanoseconds / 1e9
            if elapsed > timeout:
                raise RuntimeError('JointPositionController timed out waiting controller_manager: %s' % namespace)
            try:
                future = list_client.call_async(ListControllers.Request())
                rclpy.spin_until_future_complete(self._node, future, timeout_sec=1.0)
                res = future.result()
                found = sum(1 for c in res.controller if c.name in expected)
                if found == len(expected):
                    break
            except Exception:
                pass
            time.sleep(0.01)
        self._node.get_logger().info('JointPositionController initialized. ns: %s' % namespace)

    def set_joint_positions(self, jnt_positions):
        if len(jnt_positions) != self._num_joints:
            self._node.get_logger().warn('Command length mismatch. Expected %d' % self._num_joints)
            return
        for name, q in zip(self._joint_names, jnt_positions):
            msg = Float64()
            msg.data = float(q)
            self._cmd_pub[name].publish(msg)

    def valid_jnt_command(self, command):
        return len(command) == self._num_joints


class JointTrajectoryController(JointControllerBase):
    """
    Action-client based joint trajectory controller.
    Connects to FollowJointTrajectory action server.
    """

    def __init__(self, node, publisher_name='arm_controller', namespace='', timeout=5.0, joint_names=None):
        super().__init__(node, namespace, timeout=timeout, joint_names=joint_names)

        ns_prefix = (self.ns.rstrip('/') + '/') if self.ns != '/' else '/'
        trajectory_topic = ns_prefix + publisher_name + '/command'
        self.trajectory_pub = self._node.create_publisher(JointTrajectory, trajectory_topic, 10)

        action_name = ns_prefix + publisher_name + '/follow_joint_trajectory'
        self._client = ActionClient(self._node, FollowJointTrajectory, action_name)
        self._node.get_logger().debug('Waiting for [%s] action server' % action_name)
        if not self._client.wait_for_server(timeout_sec=timeout):
            raise RuntimeError('JointTrajectoryController timed out: %s' % action_name)
        self._node.get_logger().debug('Connected to [%s]' % action_name)

        if self._joint_names is None:
            raise RuntimeError('JointTrajectoryController timed out waiting joint_states: %s' % self.ns)

        self._goal = FollowJointTrajectory.Goal()
        self._goal.trajectory.joint_names = copy.deepcopy(self._joint_names)
        self._result = None
        self._node.get_logger().info('JointTrajectoryController initialized. ns: %s' % self.ns)

    def add_point(self, target_time, positions, velocities=None, accelerations=None):
        point = JointTrajectoryPoint()
        point.positions = list(copy.deepcopy(positions))
        point.velocities = [0.0] * self._num_joints if velocities is None else list(copy.deepcopy(velocities))
        point.accelerations = [0.0] * self._num_joints if accelerations is None else list(copy.deepcopy(accelerations))
        point.time_from_start = Duration(seconds=target_time).to_msg()
        self._goal.trajectory.points.append(point)

    def clear_points(self):
        self._goal.trajectory.points = []

    def get_num_points(self):
        return len(self._goal.trajectory.points)

    def get_result(self):
        return self._result

    def get_state(self):
        return self._goal_handle.status if self._goal_handle else None

    def set_trajectory(self, trajectory):
        self._goal.trajectory.points = copy.deepcopy(trajectory.points)

    def start(self, delay=0.1, wait=False):
        num_points = len(self._goal.trajectory.points)
        self._node.get_logger().debug('Executing Joint Trajectory with %d points' % num_points)
        if delay == 0:
            self._goal.trajectory.header.stamp = self._node.get_clock().now().to_msg()
        else:
            stamp = self._node.get_clock().now()
            stamp_ns = stamp.nanoseconds + int(delay * 1e9)
            from rclpy.time import Time
            self._goal.trajectory.header.stamp = Time(nanoseconds=stamp_ns).to_msg()

        future = self._client.send_goal_async(self._goal)
        rclpy.spin_until_future_complete(self._node, future)
        self._goal_handle = future.result()
        if wait:
            result_future = self._goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self._node, result_future)
            self._result = result_future.result().result

    def stop(self):
        if hasattr(self, '_goal_handle') and self._goal_handle:
            self._goal_handle.cancel_goal_async()

    def wait(self, timeout=15.0):
        if hasattr(self, '_goal_handle') and self._goal_handle:
            result_future = self._goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self._node, result_future, timeout_sec=timeout)
            self._result = result_future.result().result
            return True
        return False

    def start_no_action_server(self):
        """Publish trajectory directly without action feedback."""
        self.trajectory_pub.publish(self._goal.trajectory)
