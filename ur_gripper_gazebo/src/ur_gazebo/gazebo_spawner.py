import copy
import rclpy
from ament_index_python.packages import get_package_share_directory
import os
import time

from gazebo_msgs.msg import ModelStates, ModelState
from gazebo_msgs.srv import SpawnEntity, DeleteEntity


def delete_gazebo_models(node, models):
    """Delete Gazebo models via service."""
    try:
        client = node.create_client(DeleteEntity, '/delete_entity')
        if client.wait_for_service(timeout_sec=5.0):
            for m in models:
                req = DeleteEntity.Request()
                req.name = m
                future = client.call_async(req)
                rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)
    except Exception as e:
        node.get_logger().info("Delete Entity service call failed: {0}".format(e))


class GazeboModels:
    """Handle ROS2-Gazebo Classic model spawning/deletion."""

    def __init__(self, node, model_pkg):
        self._node = node
        self.loaded_models = []
        self._pub_model_state = self._node.create_publisher(ModelState, '/gazebo/set_model_state', 10)
        self._node.create_subscription(ModelStates, "/gazebo/model_states", self._gazebo_callback, 1)
        time.sleep(0.5)
        delete_gazebo_models(self._node, self.loaded_models)
        time.sleep(1.0)

        pkg_share = get_package_share_directory(model_pkg)
        self.model_path = os.path.join(pkg_share, 'models') + os.sep

    def load_models(self, models):
        for m in models:
            if m.file_type == 'urdf':
                self.load_urdf_model(m)
            elif m.file_type in ('sdf', 'string'):
                self.load_sdf_model(m)

    def _gazebo_callback(self, data):
        self.loaded_models = [name for name in data.name if name.endswith("_tmp")]

    def reset_models(self, models):
        for m in models:
            self.reset_model(m)

    def reset_model(self, model):
        m_id = (model.model_id if model.model_id is not None else model.name) + '_tmp'
        if m_id in self.loaded_models:
            delete_gazebo_models(self._node, [m_id])
            time.sleep(0.5)
        self.load_models([model])

    def update_models_state(self, models):
        for m in models:
            self.update_model_state(m)

    def update_model_state(self, model):
        m_id = (model.model_id if model.model_id is not None else model.name) + '_tmp'
        if m_id in self.loaded_models:
            model_state = ModelState()
            model_state.model_name = m_id
            model_state.pose = model.pose
            model_state.reference_frame = model.reference_frame
            for _ in range(100):
                self._pub_model_state.publish(model_state)
        else:
            self.load_models([model])

    def _spawn_entity(self, name, xml, reference_frame, pose):
        client = self._node.create_client(SpawnEntity, '/spawn_entity')
        if not client.wait_for_service(timeout_sec=5.0):
            self._node.get_logger().error("spawn_entity service not available")
            return
        req = SpawnEntity.Request()
        req.name = name
        req.xml = xml
        req.reference_frame = reference_frame
        req.initial_pose = pose
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=5.0)

    def load_urdf_model(self, model):
        try:
            m_id = (model.model_id if model.model_id is not None else model.name) + '_tmp'
            xml = self.load_xml(model.name, filetype="urdf")
            self._spawn_entity(m_id, xml, model.reference_frame, model.pose)
        except IOError:
            self.load_sdf_model(model)
        except Exception as e:
            self._node.get_logger().error("Spawn URDF failed: {0}".format(e))

    def load_sdf_model(self, model):
        try:
            m_id = (model.model_id if model.model_id is not None else model.name) + '_tmp'
            if model.string_model is None:
                xml = self.load_xml(model.name)
            else:
                xml = model.string_model
            self._spawn_entity(m_id, xml, model.reference_frame, model.pose)
        except Exception as e:
            self._node.get_logger().error("Spawn SDF failed: {0}".format(e))

    def load_xml(self, model_name, filetype="sdf"):
        with open(os.path.join(self.model_path, model_name, "model.%s" % filetype), "r") as f:
            return f.read().replace('\n', '')
