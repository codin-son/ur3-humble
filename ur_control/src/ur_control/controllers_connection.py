#!/usr/bin/env python3

import rclpy
from controller_manager_msgs.srv import SwitchController, LoadController, UnloadController, ListControllers
from ur_control.utils import solve_namespace
import time


class ControllersConnection():
    def __init__(self, node, namespace=None):
        self._node = node
        self.controllers_list = []
        self.ns = solve_namespace(namespace)[1:]  # strip leading slash

        if namespace:
            prefix = namespace.rstrip('/') + '/controller_manager/'
        else:
            prefix = '/controller_manager/'

        self.switch_service = self._node.create_client(SwitchController, prefix + 'switch_controller')
        self.load_service = self._node.create_client(LoadController, prefix + 'load_controller')
        self.unload_service = self._node.create_client(UnloadController, prefix + 'unload_controller')
        self.list_controllers_service = self._node.create_client(ListControllers, prefix + 'list_controllers')
        self.list_controllers_service.wait_for_service(timeout_sec=1.0)

    def _call_service(self, client, request, timeout=5.0):
        if not client.service_is_ready():
            client.wait_for_service(timeout_sec=timeout)
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=timeout)
        return future.result()

    def get_loaded_controllers(self):
        self.controllers_list = []
        try:
            result = self._call_service(self.list_controllers_service, ListControllers.Request())
            for controller in result.controller:
                self.controllers_list.append(controller.name)
        except Exception:
            pass

    def get_controller_state(self, controller_name):
        try:
            result = self._call_service(self.list_controllers_service, ListControllers.Request())
            for controller in result.controller:
                if controller.name == controller_name:
                    return controller.state
        except Exception:
            pass
        self._node.get_logger().error("Controller %s not found" % controller_name)
        return None

    def load_controllers(self, controllers_list):
        self.get_loaded_controllers()
        for controller in controllers_list:
            if controller not in self.controllers_list:
                req = LoadController.Request()
                req.name = controller
                result = self._call_service(self.load_service, req, timeout=0.1)
                self._node.get_logger().info('Loading controller %s. Result=%s' % (controller, result))
                if result:
                    self.controllers_list.append(controller)

    def unload_controllers(self, controllers_list):
        try:
            for controller in controllers_list:
                req = UnloadController.Request()
                req.name = controller
                result = self._call_service(self.unload_service, req, timeout=0.1)
                self._node.get_logger().info('Unloading controller %s. Result=%s' % (controller, result))
                if result:
                    self.controllers_list.remove(controller)
        except Exception as e:
            self._node.get_logger().error("Unload controllers failed: %s" % e)

    def check_controllers_state(self, on_controllers, off_controllers):
        for c in on_controllers:
            if self.get_controller_state(c) != "active":
                return False
        for c in off_controllers:
            if self.get_controller_state(c) not in ("inactive", "unconfigured"):
                return False
        return True

    def switch_controllers(self, controllers_on, controllers_off, strictness=1):
        try:
            if self.check_controllers_state(controllers_on, controllers_off):
                return True

            req = SwitchController.Request()
            req.activate_controllers = controllers_on
            req.deactivate_controllers = controllers_off
            req.strictness = strictness

            result = self._call_service(self.switch_service, req, timeout=0.1)
            self._node.get_logger().debug("Switch Result==> " + str(result.ok if result else None))

            if not result or not result.ok:
                return False

            start_time = time.time()
            while (time.time() - start_time) < 5.0:
                all_running = all(self.get_controller_state(c) == "active" for c in controllers_on)
                if all_running:
                    break

            return result.ok
        except Exception as e:
            self._node.get_logger().error("Switch controllers failed: %s" % e)
            return None

    def reset_controllers(self):
        result_off_ok = self.switch_controllers(controllers_on=[], controllers_off=self.controllers_list)
        self._node.get_logger().debug("Deactivated Controllers")
        if result_off_ok:
            result_on_ok = self.switch_controllers(controllers_on=self.controllers_list, controllers_off=[])
            if result_on_ok:
                self._node.get_logger().debug("Controllers Reset==> " + str(self.controllers_list))
                return True
        return False

    def update_controllers_list(self, new_controllers_list):
        self.controllers_list = new_controllers_list
