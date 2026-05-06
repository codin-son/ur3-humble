#!/usr/bin/env python3
"""Spawn/place models in Gazebo Classic — ROS2 Humble"""
import argparse
import rclpy
from rclpy.node import Node
import numpy as np

from ur_gazebo.gazebo_spawner import GazeboModels
from ur_gazebo.model import Model
from ur_gazebo.basic_models import SPHERE, PEG_BOARD, BOX, SPHERE_COLLISION


class SpawnerNode(Node):
    def __init__(self):
        super().__init__('gazebo_spawner_ur3e')
        self.spawner = GazeboModels(self, 'ur_gripper_gazebo')

    def place_target(self):
        sphere = SPHERE % ("target", "0.02", "GreenTransparent")
        objpose = [[-0.13101034, 0.37818616, 0.50852045], [0, 0, 0]]
        models = [Model("target", objpose[0], file_type='string', string_model=sphere, reference_frame="world")]
        self.spawner.load_models(models)

    def place_ball(self):
        sphere = SPHERE_COLLISION.format("ball", "0.1", "Yellow", "0.1", 2e5)
        objpose = [[0.608678, 0.0, (0.25939 + 0.685)], [0, 0, 0]]
        models = [Model("ball", objpose[0], file_type='string', string_model=sphere, reference_frame="world")]
        self.spawner.load_models(models)

    def place_cube(self):
        cube_length = "0.2"
        obj = BOX % ("box", cube_length, cube_length, cube_length, "Yellow", cube_length, cube_length, cube_length)
        objpose = [[0.618678, 0.0, 0.955148], [0, 0.0, 0, 0.0]]
        models = [Model("box", objpose[0], file_type='string', string_model=obj, reference_frame="world")]
        self.spawner.load_models(models)

    def place_models(self):
        model_names = ["simple_peg_board"]
        objpose = [[-0.45, -0.20, 0.86], [0, 0.1986693, 0, 0.9800666]]
        models = [Model(model_names[0], objpose[0], orientation=objpose[1])]
        self.spawner.load_models(models)

    def place_soft(self):
        name = "simple_peg_board"
        objpose = [[-0.3349516, 0.00327044, 0.45290458], [-0.50128434, -0.49779569, 0.50398595, 0.496]]
        stiffness = 1e5
        g = np.interp(stiffness, [1e4, 1e5], [0, 1]) if stiffness <= 1e5 else np.interp(stiffness, [1e5, 1e6], [1, 0])
        b = np.interp(stiffness, [1e4, 1e5], [1, 0])
        r = np.interp(stiffness, [1e5, 1e6], [0, 1])
        string_model = PEG_BOARD.format(r, g, b, stiffness)
        models = [Model(name, objpose[0], orientation=objpose[1], file_type='string', string_model=string_model, reference_frame="base_link")]
        self.spawner.load_models([models])

    def place_aruco(self):
        name = "Apriltag36_11_00000"
        objpose = [[-0.65, 0.0, 0.78], [0, 0, np.pi / 8.0]]
        models = [Model(name, objpose[0], orientation=objpose[1], file_type='sdf', reference_frame="world")]
        self.spawner.load_models(models)


def main(args=None):
    rclpy.init(args=args)
    node = SpawnerNode()

    parser = argparse.ArgumentParser(description='Gazebo model spawner')
    parser.add_argument('--place', action='store_true')
    parser.add_argument('--target', action='store_true')
    parser.add_argument('--soft', action='store_true')
    parser.add_argument('--aruco', action='store_true')
    parser.add_argument('--ball', action='store_true')
    parser.add_argument('--cube', action='store_true')
    parsed, _ = parser.parse_known_args()

    if parsed.place:
        node.place_models()
    if parsed.target:
        node.place_target()
    if parsed.soft:
        node.place_soft()
    if parsed.aruco:
        node.place_aruco()
    if parsed.ball:
        node.place_ball()
    if parsed.cube:
        node.place_cube()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
