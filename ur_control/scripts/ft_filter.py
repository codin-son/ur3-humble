#!/usr/bin/env python3
# Copyright (c) 2018-2023 Cristian Beltran — ROS2 Humble port

import argparse
import collections
import numpy as np
import time

import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty, SetBool
from geometry_msgs.msg import WrenchStamped

from ur_control import spalg, utils, filters, conversions


class FTsensorNode(Node):
    def __init__(self, namespace, in_topic, out_topic=None, republish=False,
                 sampling_frequency=500, cutoff=10, order=2, data_window=10):
        super().__init__('ft_filter')
        self.ns = namespace
        self.enable_publish = republish
        self.enable_filtering = True

        ns_prefix = (namespace.rstrip('/') + '/') if namespace else ''
        self.in_topic = '/' + ns_prefix + in_topic.lstrip('/')
        if out_topic:
            self.out_topic = '/' + ns_prefix + out_topic.lstrip('/')
            self.out_tcp_topic = self.out_topic + 'tcp'
        else:
            self.out_topic = self.in_topic.rstrip('/') + '/filtered'
            self.out_tcp_topic = self.in_topic.rstrip('/') + '/tcp'

        self.get_logger().info("Publishing filtered FT to %s" % self.out_topic)

        # Load offset param
        self.declare_parameter('ft_offset', [0.0]*6)
        ft_offset_list = self.get_parameter('ft_offset').value
        self.wrench_offset = np.array(ft_offset_list) if ft_offset_list else np.zeros(6)

        self.pub = self.create_publisher(WrenchStamped, self.out_topic, 1)
        self.pub_tcp = self.create_publisher(WrenchStamped, self.out_tcp_topic, 1)

        self.create_service(Empty, self.out_topic + '/zero_ftsensor', self._srv_zeroing)
        self.create_service(SetBool, self.out_topic + '/enable_publish', self._srv_publish)
        self.create_service(SetBool, self.out_topic + '/enable_filtering', self._srv_filtering)

        self.filter = filters.ButterLowPass(cutoff, sampling_frequency, order)
        self.data_window = data_window
        assert self.data_window >= 5
        self.data_queue = collections.deque(maxlen=self.data_window)

        self.create_subscription(WrenchStamped, self.in_topic, self.raw_wrench_cb, 1)
        self.get_logger().info('FT filter successfully initialized')

    def add_wrench_observation(self, wrench):
        self.data_queue.append(np.array(wrench))

    def raw_wrench_cb(self, msg):
        current_wrench = conversions.from_wrench(msg.wrench)
        self.add_wrench_observation(current_wrench)
        if self.enable_publish:
            if self.enable_filtering:
                current_wrench = self.get_filtered_wrench()
            if current_wrench is not None:
                data = current_wrench - self.wrench_offset
                out_msg = WrenchStamped()
                out_msg.header = msg.header
                out_msg.wrench = conversions.to_wrench(data)
                self.pub.publish(out_msg)

                tcp_param = self.out_tcp_topic + '/pose_sensor_to_tcp'
                if self.has_parameter(tcp_param):
                    pose_sensor_to_tcp = self.get_parameter(tcp_param).value
                    tcp_wrench = data.copy()
                    tcp_wrench[:3] += spalg.sensor_torque_to_tcp_force(
                        tcp_position=pose_sensor_to_tcp, sensor_torques=current_wrench[3:])
                    tcp_wrench[3:] = np.zeros(3)
                    tcp_msg = WrenchStamped()
                    tcp_msg.header = msg.header
                    tcp_msg.wrench = conversions.to_wrench(tcp_wrench)
                    self.pub_tcp.publish(tcp_msg)

    def get_filtered_wrench(self):
        if len(self.data_queue) < self.data_window:
            return None
        return self.filter(np.array(self.data_queue))[-1, :]

    def update_wrench_offset(self):
        current_wrench = self.get_filtered_wrench()
        if current_wrench is not None:
            self.wrench_offset = current_wrench
            self.set_parameters([
                rclpy.parameter.Parameter('ft_offset', value=self.wrench_offset.tolist())
            ])

    def _srv_zeroing(self, request, response):
        self.update_wrench_offset()
        return response

    def _srv_publish(self, request, response):
        self.enable_publish = request.data
        response.success = True
        return response

    def _srv_filtering(self, request, response):
        self.enable_filtering = request.data
        response.success = True
        return response


def main(args=None):
    parser = argparse.ArgumentParser(description='Filter FT signal')
    parser.add_argument('-ns', '--namespace', type=str, default="")
    parser.add_argument('-t', '--ft_topic', type=str, required=True)
    parser.add_argument('-ot', '--out_topic', type=str, default=None)
    parser.add_argument('-z', '--zero', action='store_true')
    parsed, remaining = parser.parse_known_args()

    rclpy.init(args=remaining)
    node = FTsensorNode(
        namespace=parsed.namespace,
        in_topic=parsed.ft_topic,
        out_topic=parsed.out_topic,
        republish=True)
    if parsed.zero:
        time.sleep(1.0)  # wait to fill filter
        node.update_wrench_offset()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
