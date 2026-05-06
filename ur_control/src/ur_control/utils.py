# ROS2 utilities used by the CRI group
import os
import sys
import copy
import time
import numpy as np
import rclpy
import rclpy.logging
from ament_index_python.packages import get_package_share_directory
from ur_control import transformations, spalg
from sensor_msgs.msg import JointState
from pyquaternion import Quaternion


def load_urdf_string(package, filename):
    package_dir = get_package_share_directory(package)
    urdf_file = os.path.join(package_dir, 'urdf', filename + '.urdf')
    with open(urdf_file) as f:
        return f.read()


class PDRotation:
    def __init__(self, kp, kd=None):
        self.kp = np.array(kp)
        self.kd = np.array(kd)
        self.reset()

    def reset(self):
        self.last_time = time.time()
        self.last_error = Quaternion()

    def set_gains(self, kp=None, kd=None):
        if kp is not None:
            self.kp = np.array(kp)
        if kd is not None:
            self.kd = np.array(kd)

    def update(self, quaternion_error, dt=None):
        now = time.time()
        if dt is None:
            dt = now - self.last_time

        k_prime = 2 * quaternion_error.scalar * np.identity(3) - spalg.skew(quaternion_error.vector)
        p_term = np.dot(self.kp, k_prime)

        w = transformations.angular_velocity_from_quaternions(quaternion_error, self.last_error, dt)
        d_term = self.kd * w

        output = p_term + d_term
        self.last_error = quaternion_error
        self.last_time = now
        return output


class PID:
    def __init__(self, Kp, Ki=None, Kd=None, dynamic_pid=False, max_gain_multiplier=10.0):
        self.Kp = np.array(Kp)
        self.Ki = np.zeros_like(Kp)
        self.Kd = np.zeros_like(Kp)
        if Ki is not None:
            self.Ki = np.array(Ki)
        if Kd is not None:
            self.Kd = np.array(Kd)
        self.set_windup(np.ones_like(self.Kp))
        self.reset()
        self.scale_gains = dynamic_pid
        self.max_gain_multiplier = max_gain_multiplier

    def reset(self):
        self.last_time = time.time()
        self.last_error = np.zeros_like(self.Kp)
        self.integral = np.zeros_like(self.Kp)

    def set_gains(self, Kp=None, Ki=None, Kd=None):
        if Kp is not None:
            self.Kp = np.array(Kp)
        if Ki is not None:
            self.Ki = np.array(Ki)
        if Kd is not None:
            self.Kd = np.array(Kd)

    def set_windup(self, windup):
        self.i_min = -np.array(windup)
        self.i_max = np.array(windup)

    def update(self, error, dt=None):
        if self.scale_gains:
            kp = np.zeros_like(self.Kp)
            kd = np.zeros_like(self.Kd)
            for i in range(len(error)):
                factor = 1 - np.tanh(100 * error[i])
                kp[i] = np.interp(factor, [0.0, 1.0], [self.Kp[i], self.Kp[i] * self.max_gain_multiplier])
                kd[i] = np.interp(factor, [0.0, 1.0], [self.Kd[i], self.Kd[i] * self.max_gain_multiplier])
            kd = self.Kd
            ki = self.Ki
        else:
            kp = self.Kp
            kd = self.Kd
            ki = self.Ki

        now = time.time()
        if dt is None:
            dt = now - self.last_time
        delta_error = error - self.last_error
        self.integral += error * dt
        p_term = kp * error
        i_term = ki * self.integral
        i_term = np.maximum(self.i_min, np.minimum(i_term, self.i_max))

        if not np.allclose(self.last_error, np.zeros_like(self.last_error)):
            d_term = kd * delta_error / dt
        else:
            d_term = kd * np.zeros_like(delta_error) / dt

        output = p_term + i_term + d_term
        self.last_error = np.array(error)
        self.last_time = now
        return output


class TextColors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    log_level = rclpy.logging.LoggingSeverity.INFO

    def disable(self):
        self.HEADER = ''
        self.OKBLUE = ''
        self.OKGREEN = ''
        self.WARNING = ''
        self.FAIL = ''
        self.ENDC = ''

    def blue(self, msg):
        print((self.OKBLUE + msg + self.ENDC))

    def debug(self, msg):
        print((self.OKGREEN + msg + self.ENDC))

    def error(self, msg):
        print((self.FAIL + msg + self.ENDC))

    def ok(self, msg):
        print((self.OKGREEN + msg + self.ENDC))

    def warning(self, msg):
        print((self.WARNING + msg + self.ENDC))

    def logdebug(self, msg):
        if self.log_level <= rclpy.logging.LoggingSeverity.DEBUG:
            print((self.OKGREEN + 'Debug ' + self.ENDC + str(msg)))

    def loginfo(self, msg):
        if self.log_level <= rclpy.logging.LoggingSeverity.INFO:
            print(('INFO ' + str(msg)))

    def logwarn(self, msg):
        if self.log_level <= rclpy.logging.LoggingSeverity.WARN:
            print((self.WARNING + 'Warning ' + self.ENDC + str(msg)))

    def logerr(self, msg):
        if self.log_level <= rclpy.logging.LoggingSeverity.ERROR:
            print((self.FAIL + 'Error ' + self.ENDC + str(msg)))

    def logfatal(self, msg):
        if self.log_level <= rclpy.logging.LoggingSeverity.FATAL:
            print((self.FAIL + 'Fatal ' + self.ENDC + str(msg)))

    def set_log_level(self, level):
        self.log_level = level


## Helper Functions ##
def assert_shape(variable, name, shape):
    assert variable.shape == shape, '%s must have a shape %r: %r' % (name, shape, variable.shape)


def assert_type(variable, name, ttype):
    assert type(variable) is ttype, '%s must be of type %r: %r' % (name, ttype, type(variable))


def db_error_msg(name, logger=TextColors()):
    msg = 'Database %s not found. Please generate it.' % name
    logger.logerr(msg)


def clean_cos(value):
    return min(1, max(value, -1))


def has_keys(data, keys):
    if not isinstance(data, dict):
        return False
    return all(k in data for k in keys)


def raise_not_implemented():
    raise NotImplementedError()


def topic_exist(node, topic):
    """Check if topic exists. node: rclpy.Node instance."""
    topic_names = [name for name, _ in node.get_topic_names_and_types()]
    return topic in topic_names


def read_key(echo=False):
    if not echo:
        os.system("stty -echo")
    key = sys.stdin.read(1)
    if not echo:
        os.system("stty echo")
    return key.lower()


def resolve_parameter(value, default_value):
    if value:
        return value
    return default_value


def read_parameter(node, name, default):
    """Get ROS2 parameter. node: rclpy.Node instance."""
    if not node.has_parameter(name):
        node.get_logger().warn('Parameter [%s] not found, using default: %s' % (name, default))
    return node.get_parameter_or(name, rclpy.parameter.Parameter(name, value=default)).value


def read_parameter_err(node, name):
    """Get ROS2 parameter or log error."""
    if not node.has_parameter(name):
        node.get_logger().error("Parameter [%s] not found" % name)
        return False, None
    return True, node.get_parameter(name).value


def read_parameter_fatal(node, name):
    """Get ROS2 parameter or raise."""
    if not node.has_parameter(name):
        node.get_logger().fatal("Parameter [%s] not found" % name)
        raise Exception('Required parameter {0} not found'.format(name))
    return node.get_parameter(name).value


def solve_namespace(namespace=None):
    """Normalize ROS2 namespace string."""
    if namespace is None or len(namespace) == 0:
        return '/'
    if not namespace.startswith('/'):
        namespace = '/' + namespace
    if not namespace.endswith('/'):
        namespace += '/'
    return namespace


def sorted_joint_state_msg(msg, joint_names):
    valid_names = set(joint_names).intersection(set(msg.name))
    valid_position = len(msg.name) == len(msg.position)
    valid_velocity = len(msg.name) == len(msg.velocity)
    valid_effort = len(msg.name) == len(msg.effort)
    retmsg = JointState()
    retmsg.header = copy.deepcopy(msg.header)
    for name in joint_names:
        if name not in valid_names:
            continue
        idx = msg.name.index(name)
        retmsg.name.append(name)
        if valid_position:
            retmsg.position.append(msg.position[idx])
        if valid_velocity:
            retmsg.velocity.append(msg.velocity[idx])
        if valid_effort:
            retmsg.effort.append(msg.effort[idx])
    return retmsg


def unique(data):
    order = np.lexsort(data.T)
    data = data[order]
    diff = np.diff(data, axis=0)
    ui = np.ones(len(data), 'bool')
    ui[1:] = (diff != 0).any(axis=1)
    return data[ui]


def wait_for(predicate, timeout=5.0):
    start_time = time.time()
    while not predicate():
        now = time.time()
        if (now - start_time) > timeout:
            return False
        time.sleep(0.001)
    return True
