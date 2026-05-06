# Gripper action client — ROS2 Humble
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
import numpy as np
from sensor_msgs.msg import JointState
from control_msgs.action import GripperCommand

from ur_control import utils

try:
    from gazebo_ros_link_attacher.srv import Attach
except ImportError:
    print("Grasping plugin can't be loaded")

try:
    import robotiq_msgs.msg
except ImportError:
    print("Robotiq gripper can't be loaded. robotiq_msgs required.")


class GripperControllerBase():
    def __init__(self, node, namespace='', node_name='', prefix=None, timeout=5.0) -> None:
        self._node = node
        self.ns = utils.solve_namespace(namespace)
        self.prefix = prefix if prefix is not None else ''
        self.valid_joint_names = []
        self._joint_names = None
        self._current_jnt_positions = None
        self._current_jnt_velocities = None
        self._current_jnt_efforts = None

        # Read gripper joint names from parameters
        ns_prefix = self.ns.rstrip('/') + '/' + node_name + '/'
        try:
            for param_name in ('joint', 'joints', 'joint_name'):
                full_name = ns_prefix + param_name
                if self._node.has_parameter(full_name):
                    val = self._node.get_parameter(full_name).value
                    if isinstance(val, str):
                        self.valid_joint_names = [prefix + val if prefix else val]
                    else:
                        self.valid_joint_names = val
                    break
            else:
                self._node.get_logger().error("Couldn't find valid joints params in %s" % ns_prefix)
                return
        except Exception as e:
            self._node.get_logger().error("Error reading gripper params: %s" % str(e))
            return

        self._js_sub = self._node.create_subscription(
            JointState, '/joint_states', self.joint_states_cb, 1)

        import time
        start_time = time.time()
        retry = False
        while self._joint_names is None:
            elapsed = time.time() - start_time
            if elapsed > timeout and not retry:
                topic = (self.ns.rstrip('/') + '/joint_states')
                self._js_sub = self._node.create_subscription(
                    JointState, topic, self.joint_states_cb, 1)
                start_time = time.time()
                retry = True
                continue
            elif elapsed > timeout and retry:
                self._node.get_logger().error('Timed out waiting for gripper joint_states topic')
                return
            rclpy.spin_once(self._node, timeout_sec=0.01)

    def open(self):
        raise NotImplementedError()

    def close(self):
        raise NotImplementedError()

    def get_position(self):
        return self._current_jnt_positions[0]

    def get_velocity(self):
        return self._current_jnt_velocities[0]

    def get_opening_percentage(self):
        raise NotImplementedError()

    def joint_states_cb(self, msg):
        position = []
        velocity = []
        effort = []
        name = []
        for joint_name in self.valid_joint_names:
            if joint_name in msg.name:
                idx = msg.name.index(joint_name)
                name.append(msg.name[idx])
                effort.append(msg.effort[idx])
                velocity.append(msg.velocity[idx])
                position.append(msg.position[idx])
        if set(name) == set(self.valid_joint_names):
            self._current_jnt_positions = np.array(position)
            self._current_jnt_velocities = np.array(velocity)
            self._current_jnt_efforts = np.array(effort)
            self._joint_names = list(name)


class GripperController(GripperControllerBase):
    def __init__(self, node, namespace='', prefix=None, timeout=5.0, attach_link='robot::wrist_3_link'):
        node_name = "gripper_controller"
        super().__init__(node, namespace, node_name, prefix, timeout)

        ns_prefix = self.ns.rstrip('/') + '/' + node_name + '/'
        self.gripper_type = str(self._node.get_parameter_or(
            ns_prefix + 'gripper_type',
            rclpy.parameter.Parameter('gripper_type', value='85')).value)

        if self.gripper_type == "hand-e":
            self._max_gap = 0.025 * 2.0
            self._to_open = 0.0
            self._to_close = self._max_gap
        elif self.gripper_type == "85":
            self._max_gap = 0.085
            self._to_open = self._max_gap
            self._to_close = 0.001
            self._max_angle = 0.8028
        elif self.gripper_type == "140":
            self._max_gap = 0.140
            self._to_open = self._max_gap
            self._to_close = 0.001
            self._max_angle = 0.69

        attach_plugin = self._node.get_parameter_or(
            'grasp_plugin',
            rclpy.parameter.Parameter('grasp_plugin', value=False)).value
        if attach_plugin:
            try:
                self.attach_link = attach_link
                self.attach_srv = self._node.create_client(Attach, '/link_attacher_node/attach')
                self.detach_srv = self._node.create_client(Attach, '/link_attacher_node/detach')
                self.attach_srv.wait_for_service(timeout_sec=5.0)
                self.detach_srv.wait_for_service(timeout_sec=5.0)
            except Exception:
                self._node.get_logger().error("Failed to load grasp plugin services.")

        action_name = (self.ns.rstrip('/') + '/' + node_name + '/gripper_cmd')
        self._client = ActionClient(self._node, GripperCommand, action_name)
        self._goal = GripperCommand.Goal()
        if not self._client.wait_for_server(timeout_sec=timeout):
            raise RuntimeError('GripperCommandAction timed out: %s' % action_name)
        self._node.get_logger().info('GripperCommandAction initialized. ns: %s' % self.ns)
        self._result = None

    def _send_goal_sync(self, wait=True):
        future = self._client.send_goal_async(self._goal)
        rclpy.spin_until_future_complete(self._node, future)
        goal_handle = future.result()
        if wait:
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self._node, result_future, timeout_sec=2.0)
            self._result = result_future.result()
        import time
        time.sleep(0.05)

    def close(self, wait=True):
        return self.command(0.0, percentage=True, wait=wait)

    def percentage_command(self, value, wait=True):
        return self.command(value, percentage=True, wait=wait)

    def command(self, value, percentage=False, wait=True):
        if value == "close":
            return self.close()
        elif value == "open":
            return self.open()

        if self.gripper_type in ("85", "140"):
            if percentage:
                value = np.clip(value, 0.0, 1.0)
                cmd = value * self._max_gap
            else:
                cmd = np.clip(value, 0.0, self._max_gap)
            angle = self._distance_to_angle(cmd)
            self._goal.command.position = angle
        if self.gripper_type == "hand-e":
            if percentage:
                value = np.clip(value, 0.0, 1.0)
                cmd = (1.0 - value) * self._max_gap / 2.0
            else:
                cmd = np.clip(value, 0.0, self._max_gap)
                cmd = (self._max_gap - value) / 2.0
            self._goal.command.position = cmd
        self._send_goal_sync(wait=wait)
        return True

    def _distance_to_angle(self, distance):
        distance = np.clip(distance, 0, self._max_gap)
        return (self._max_gap - distance) * self._max_angle / self._max_gap

    def _angle_to_distance(self, angle):
        angle = np.clip(angle, 0, self._max_angle)
        return (self._max_angle - angle) * self._max_gap / self._max_angle

    def get_result(self):
        return self._result

    def grab(self, link_name):
        parent = self.attach_link.split('::')
        child = link_name.split('::')
        req = Attach.Request()
        req.model_name_1 = parent[0]
        req.link_name_1 = parent[1]
        req.model_name_2 = child[0]
        req.link_name_2 = child[1]
        future = self.attach_srv.call_async(req)
        rclpy.spin_until_future_complete(self._node, future)
        return future.result().ok

    def open(self, wait=True):
        return self.command(1.0, percentage=True, wait=wait)

    def release(self, link_name):
        parent = self.attach_link.rsplit('::')
        child = link_name.rsplit('::')
        req = Attach.Request()
        req.model_name_1 = parent[0]
        req.link_name_1 = parent[1]
        req.model_name_2 = child[0]
        req.link_name_2 = child[1]
        future = self.detach_srv.call_async(req)
        rclpy.spin_until_future_complete(self._node, future)
        return future.result().ok

    def stop(self):
        pass  # Cancel via goal handle if needed

    def get_position(self):
        if self.gripper_type == "hand-e":
            return self._max_gap - (self._current_jnt_positions[0] * 2.0)
        else:
            return self._angle_to_distance(self._current_jnt_positions[0])

    def get_opening_percentage(self):
        return self.get_position() / self._max_gap


class RobotiqGripper(GripperControllerBase):
    def __init__(self, node, namespace="", prefix="", timeout=2):
        node_name = "gripper_action_controller"
        super().__init__(node, namespace, node_name, prefix, timeout)
        self.ns = namespace
        self.opening_width = 0.0

        self._gripper_client = ActionClient(
            self._node,
            robotiq_msgs.msg.CModelCommandAction,
            (self.ns.rstrip('/') + '/' if self.ns else '') + 'gripper_action_controller')

        ns_prefix = (namespace.rstrip('/') + '/' if namespace else '')
        self._gripper_status_sub = self._node.create_subscription(
            robotiq_msgs.msg.CModelCommandFeedback,
            '/%sgripper_status' % ns_prefix,
            self._gripper_status_callback, 1)

        self.gripper_type = "finger_joint"
        self._max_gap = 0.085
        self._max_angle = 0.8

        param_name = ns_prefix + 'gripper_action_controller/joint_name'
        if self._node.has_parameter(param_name):
            self.gripper_type = self._node.get_parameter(param_name).value
            self._max_gap = float(self._node.get_parameter(ns_prefix + 'gripper_action_controller/max_gap').value)
            self._max_angle = float(self._node.get_parameter(ns_prefix + 'gripper_action_controller/counts_to_meters').value)

        if self.gripper_type == "robotiq_hande_joint_finger":
            self._to_open = 0.0
            self._to_close = self._max_gap
        elif self.gripper_type == "finger_joint":
            self._to_open = self._max_gap
            self._to_close = 0.001

        if self._gripper_client.wait_for_server(timeout_sec=float(timeout)):
            self._node.get_logger().info("=== Connected to ROBOTIQ gripper ===")
        else:
            self._node.get_logger().error("Unable to connect to ROBOTIQ gripper")

    def _gripper_status_callback(self, msg):
        self.opening_width = msg.position

    def get_opening_percentage(self):
        return self.opening_width / self._max_gap

    def close(self, force=40.0, velocity=1.0, wait=True):
        return self.command("close", force=force, velocity=velocity, wait=wait)

    def open(self, velocity=1.0, wait=True, opening_width=None):
        command = opening_width if opening_width else "open"
        return self.command(command, wait=wait, velocity=velocity)

    def percentage_command(self, value, wait=True):
        value = np.clip(value, 0.0, 1.0)
        cmd = value * self._max_gap
        return self.command(cmd, wait=wait)

    def command(self, command, force=40.0, velocity=1.0, wait=True):
        goal = robotiq_msgs.msg.CModelCommandGoal()
        goal.velocity = velocity
        goal.force = force
        if command == "close":
            goal.position = 0.0
        elif command == "open":
            goal.position = 0.140
        else:
            goal.position = command

        future = self._gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self._node, future)
        goal_handle = future.result()
        self._node.get_logger().debug("Sending command " + str(command) + " to gripper: " + str(self.ns))
        if wait:
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self._node, result_future, timeout_sec=5.0)
            result = result_future.result()
            return True if result else False
        return True
