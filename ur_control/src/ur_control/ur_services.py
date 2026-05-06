
import rclpy
import time

import controller_manager_msgs.srv
import std_srvs.srv
from ur_control import conversions
from ur_control.utils import solve_namespace
import ur_dashboard_msgs.srv
import ur_msgs.srv

from std_msgs.msg import Bool


def check_for_real_robot(func):
    '''Decorator that validates the real robot is used'''
    def wrap(*args, **kwargs):
        if args[0].use_real_robot:
            return func(*args, **kwargs)
        args[0]._node.get_logger().debug("Ignoring function %s since no real robot is being used" % func.__name__)
        return True
    return wrap


class URServices():
    """Universal Robots driver specific services — ROS2 Humble"""

    def __init__(self, node, namespace):
        self._node = node
        self.use_real_robot = False
        if self._node.has_parameter('use_real_robot'):
            self.use_real_robot = self._node.get_parameter('use_real_robot').value

        self.ns = solve_namespace(namespace)
        ns = self.ns.rstrip('/')

        self.ur_ros_control_running_on_robot = False
        self.robot_safety_mode = None
        self.robot_status = {}

        def make_client(srv_type, name):
            return self._node.create_client(srv_type, name)

        self.ur_dashboard_clients = {
            "get_loaded_program":     make_client(ur_dashboard_msgs.srv.GetLoadedProgram, ns + '/ur_hardware_interface/dashboard/get_loaded_program'),
            "program_running":        make_client(ur_dashboard_msgs.srv.IsProgramRunning, ns + '/ur_hardware_interface/dashboard/program_running'),
            "load_program":           make_client(ur_dashboard_msgs.srv.Load, ns + '/ur_hardware_interface/dashboard/load_program'),
            "play":                   make_client(std_srvs.srv.Trigger, ns + '/ur_hardware_interface/dashboard/play'),
            "stop":                   make_client(std_srvs.srv.Trigger, ns + '/ur_hardware_interface/dashboard/stop'),
            "quit":                   make_client(std_srvs.srv.Trigger, ns + '/ur_hardware_interface/dashboard/quit'),
            "connect":                make_client(std_srvs.srv.Trigger, ns + '/ur_hardware_interface/dashboard/connect'),
            "close_popup":            make_client(std_srvs.srv.Trigger, ns + '/ur_hardware_interface/dashboard/close_popup'),
            "unlock_protective_stop": make_client(std_srvs.srv.Trigger, ns + '/ur_hardware_interface/dashboard/unlock_protective_stop'),
            "is_in_remote_control":   make_client(ur_dashboard_msgs.srv.IsInRemoteControl, ns + '/ur_hardware_interface/dashboard/is_in_remote_control'),
        }

        self.set_payload_srv = make_client(ur_msgs.srv.SetPayload, ns + '/ur_hardware_interface/set_payload')
        self.speed_slider = make_client(ur_msgs.srv.SetSpeedSliderFraction, ns + '/ur_hardware_interface/set_speed_slider')
        self.set_io = make_client(ur_msgs.srv.SetIO, ns + '/ur_hardware_interface/set_io')

        self.sub_status_ = self._node.create_subscription(
            Bool, ns + '/ur_hardware_interface/robot_program_running',
            self.ros_control_status_callback, 1)
        self.service_proxy_list = make_client(
            controller_manager_msgs.srv.ListControllers, ns + '/controller_manager/list_controllers')
        self.service_proxy_switch = make_client(
            controller_manager_msgs.srv.SwitchController, ns + '/controller_manager/switch_controller')
        self.sub_robot_safety_mode = self._node.create_subscription(
            ur_dashboard_msgs.msg.SafetyMode,
            ns + '/ur_hardware_interface/safety_mode',
            self.safety_mode_callback, 1)

    def _call(self, client, request, timeout=5.0):
        if not client.service_is_ready():
            client.wait_for_service(timeout_sec=timeout)
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=timeout)
        return future.result()

    @check_for_real_robot
    def safety_mode_callback(self, msg):
        self.robot_safety_mode = msg.mode

    @check_for_real_robot
    def ros_control_status_callback(self, msg):
        self.ur_ros_control_running_on_robot = msg.data

    @check_for_real_robot
    def is_running_normally(self):
        return self.robot_safety_mode in (1, 2)

    @check_for_real_robot
    def is_protective_stopped(self):
        return self.robot_safety_mode == 3

    @check_for_real_robot
    def unlock_protective_stop(self):
        client = self.ur_dashboard_clients["unlock_protective_stop"]
        start_time = time.time()
        self._node.get_logger().info("Attempting to unlock protective stop of " + self.ns)
        response = None
        while rclpy.ok():
            response = self._call(client, std_srvs.srv.Trigger.Request())
            if time.time() - start_time > 20.0:
                self._node.get_logger().error("Timeout of 20s exceeded in unlock protective stop")
                break
            if response and response.success:
                break
            time.sleep(0.2)
        self._call(self.ur_dashboard_clients["stop"], std_srvs.srv.Trigger.Request())
        if not response or not response.success:
            self._node.get_logger().warn("Could not unlock protective stop of " + self.ns + "!")
        return response.success if response else False

    @check_for_real_robot
    def set_speed_scale(self, scale):
        try:
            req = ur_msgs.srv.SetSpeedSliderFraction.Request()
            req.speed_slider_fraction = scale
            self._call(self.speed_slider, req)
        except Exception:
            self._node.get_logger().error("Failed to communicate with Dashboard when setting speed slider")
            return False

    @check_for_real_robot
    def set_payload(self, mass, center_of_gravity):
        self.activate_ros_control_on_ur()
        try:
            req = ur_msgs.srv.SetPayload.Request()
            req.payload = mass
            req.center_of_gravity = conversions.to_vector3(center_of_gravity)
            self._call(self.set_payload_srv, req)
            return True
        except Exception as e:
            self._node.get_logger().error("Exception trying to set payload: %s" % e)
        return False

    @check_for_real_robot
    def wait_for_control_status_to_turn_on(self, waittime):
        start = time.time()
        while not self.ur_ros_control_running_on_robot and (time.time() - start) < waittime and rclpy.ok():
            time.sleep(0.1)
            if self.ur_ros_control_running_on_robot:
                return True
        return False

    @check_for_real_robot
    def activate_ros_control_on_ur(self, recursion_depth=0):
        if self.ur_ros_control_running_on_robot:
            self.set_speed_scale(scale=1.0)
            return True
        else:
            self._node.get_logger().info("Robot program not running for " + self.ns)

        try:
            response = self._call(self.ur_dashboard_clients["is_in_remote_control"], ur_dashboard_msgs.srv.IsInRemoteControl.Request())
            if not response.in_remote_control:
                self._node.get_logger().error("Unable to automatically activate robot. Manually activate via polyscope or remote control mode.")
                return False
        except Exception:
            pass

        self._node.get_logger().warn(f"Attempt to reconnect # {recursion_depth+1}")

        if recursion_depth > 10:
            raise Exception("Could not activate ROS control on robot " + self.ns)

        if not rclpy.ok():
            return False

        program_loaded = self.check_loaded_program()
        if program_loaded:
            try:
                self._call(self.ur_dashboard_clients["play"], std_srvs.srv.Trigger.Request())
            except Exception:
                pass
            if self.wait_for_control_status_to_turn_on(2.0):
                return True
            else:
                self._node.get_logger().warn("Failed to start program")

        self._node.get_logger().warn("Trying to reconnect dashboard client and then activating again.")
        try:
            self._call(self.ur_dashboard_clients["quit"], std_srvs.srv.Trigger.Request())
            time.sleep(1)
            response = self._call(self.ur_dashboard_clients["connect"], std_srvs.srv.Trigger.Request())
            time.sleep(1)
            if response and response.success:
                self._call(self.ur_dashboard_clients["stop"], std_srvs.srv.Trigger.Request())
                time.sleep(1)
                self._call(self.ur_dashboard_clients["play"], std_srvs.srv.Trigger.Request())
                time.sleep(1)
        except Exception:
            self._node.get_logger().warn("Dashboard service did not respond!")

        if self.wait_for_control_status_to_turn_on(2.0):
            if self.check_for_dead_controller_and_force_start():
                self._node.get_logger().info("Successfully activated ROS control on robot " + self.ns)
                self.set_speed_scale(scale=1.0)
                return True
        else:
            return self.activate_ros_control_on_ur(recursion_depth=recursion_depth+1)

    @check_for_real_robot
    def check_loaded_program(self):
        try:
            response = self._call(self.ur_dashboard_clients["get_loaded_program"], ur_dashboard_msgs.srv.GetLoadedProgram.Request())
            if response.program_name == '/programs/ROS_external_control.urp':
                return True
            else:
                self._node.get_logger().info("Currently loaded: " + response.program_name)
                self._node.get_logger().info("Loading ROS control on robot " + self.ns)
                req = ur_dashboard_msgs.srv.Load.Request()
                req.filename = "ROS_external_control.urp"
                response = self._call(self.ur_dashboard_clients["load_program"], req)
                if response and response.success:
                    return True
                for _ in range(10):
                    time.sleep(0.2)
                    response = self._call(self.ur_dashboard_clients["get_loaded_program"], ur_dashboard_msgs.srv.GetLoadedProgram.Request())
                    if response.program_name == '/programs/ROS_external_control.urp':
                        break
        except Exception:
            self._node.get_logger().warn("Dashboard service did not respond!")
        return False

    @check_for_real_robot
    def check_for_dead_controller_and_force_start(self):
        req = controller_manager_msgs.srv.ListControllers.Request()
        self._node.get_logger().info("Checking for dead controllers for robot " + self.ns)
        list_res = self._call(self.service_proxy_list, req)
        for c in list_res.controller:
            if c.name == "scaled_pos_joint_traj_controller":
                if c.state in ("inactive", "unconfigured"):
                    self._node.get_logger().warn("Force restart of controller")
                    switch_req = controller_manager_msgs.srv.SwitchController.Request()
                    switch_req.activate_controllers = ['scaled_pos_joint_traj_controller']
                    switch_req.strictness = 1
                    switch_res = self._call(self.service_proxy_switch, switch_req)
                    time.sleep(1)
                    return switch_res.ok if switch_res else False
                else:
                    self._node.get_logger().info("Controller state is " + c.state)
                    return True

    @check_for_real_robot
    def load_and_execute_program(self, program_name="", recursion_depth=0, skip_ros_activation=False):
        if not skip_ros_activation:
            self.activate_ros_control_on_ur()
        if not self.load_program(program_name, recursion_depth):
            return False
        return self.execute_loaded_program()

    @check_for_real_robot
    def load_program(self, program_name="", recursion_depth=0):
        if recursion_depth > 10:
            self._node.get_logger().error("Tried too often. Could not load " + program_name)
            return False

        try:
            self._call(self.ur_dashboard_clients["stop"], std_srvs.srv.Trigger.Request())
            time.sleep(0.5)
            response = self._call(self.ur_dashboard_clients["get_loaded_program"], ur_dashboard_msgs.srv.GetLoadedProgram.Request())
            if response.program_name == '/programs/' + program_name:
                return True
            else:
                req = ur_dashboard_msgs.srv.Load.Request()
                req.filename = program_name
                response = self._call(self.ur_dashboard_clients["load_program"], req)
                if response and response.success:
                    return True
                self._node.get_logger().error("Could not load " + program_name)
        except Exception:
            self._node.get_logger().warn("Dashboard service did not respond to load_program!")

        self._node.get_logger().warn("Waiting and trying again")
        time.sleep(3)
        try:
            if recursion_depth > 0:
                self._call(self.ur_dashboard_clients["quit"], std_srvs.srv.Trigger.Request())
                time.sleep(0.5)
        except Exception:
            pass
        self._call(self.ur_dashboard_clients["connect"], std_srvs.srv.Trigger.Request())
        time.sleep(0.5)
        return self.load_program(program_name=program_name, recursion_depth=recursion_depth+1)

    @check_for_real_robot
    def execute_loaded_program(self):
        try:
            response = self._call(self.ur_dashboard_clients["play"], std_srvs.srv.Trigger.Request())
            if not response or not response.success:
                self._node.get_logger().error("Could not start program.")
                return False
            self._node.get_logger().info("Successfully started program on robot " + self.ns)
            return True
        except Exception as e:
            self._node.get_logger().error(str(e))
            return False

    @check_for_real_robot
    def close_ur_popup(self):
        response = self._call(self.ur_dashboard_clients["close_popup"], std_srvs.srv.Trigger.Request())
        if not response or not response.success:
            self._node.get_logger().error("Could not close popup.")
            return False
        self._node.get_logger().info("Successfully closed popup on teach pendant of robot " + self.ns)
        return True
