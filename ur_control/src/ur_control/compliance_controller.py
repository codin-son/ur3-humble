# Copyright (c) 2018-2021, Cristian Beltran.  All rights reserved.

import rclpy
import numpy as np
import types
import time

from ur_control.arm import Arm
from ur_control import transformations, spalg, utils
from ur_control.constants import ExecutionResult


def getAvg(prev_avg, x, n):
    return ((prev_avg * n + x) / (n + 1))


class CompliantController(Arm):
    def __init__(self, model, **kwargs):
        """Compliant controller."""
        Arm.__init__(self, **kwargs)
        self.model = model
        # Rate in seconds per cycle
        js_rate = 500.0  # default; override via parameter if needed
        self.rate_period = 1.0 / js_rate

    def _rate_sleep(self):
        time.sleep(self.rate_period)

    def set_hybrid_control_trajectory(self, trajectory, max_force_torque, timeout=5.0,
                                      stop_on_target_force=False, termination_criteria=None,
                                      displacement_epsilon=0.002, check_displacement_time=2.0,
                                      verbose=True, debug=False, time_compensation=True):
        reduced_speed = np.deg2rad([100, 100, 100, 250, 250, 250])

        xb = self.end_effector()
        failure_counter = 0
        ptp_index = 0
        q_last = self.joint_angles()

        trajectory_time_compensation = self.model.dt * 10. if time_compensation else 0.0

        if trajectory.ndim == 1:
            ptp_timeout = timeout
            self.model.set_goals(position=trajectory)
        else:
            ptp_timeout = timeout / float(len(trajectory)) - trajectory_time_compensation
            self.model.set_goals(position=trajectory[ptp_index])

        log = {ExecutionResult.SPEED_LIMIT_EXCEEDED: 0, ExecutionResult.IK_NOT_FOUND: 0}
        result = ExecutionResult.DONE

        standby_timer = time.time()
        standby_last_pose = self.end_effector()
        standby = False

        initime = time.time()
        sub_inittime = time.time()

        while rclpy.ok() and (time.time() - initime) < timeout:
            if debug:
                start_time = time.time()

            Wb = self.get_wrench(base_frame_control=True)
            xb = self.end_effector()

            if termination_criteria is not None:
                assert isinstance(termination_criteria, types.LambdaType)
                if termination_criteria(xb, standby):
                    self._node.get_logger().info("Termination criteria returned True, stopping force control")
                    result = ExecutionResult.TERMINATION_CRITERIA
                    break

            if (time.time() - sub_inittime) > ptp_timeout:
                sub_inittime = time.time()
                ptp_index += 1
                if ptp_index >= len(trajectory):
                    self.model.set_goals(position=trajectory[-1])
                elif not trajectory.ndim == 1:
                    self.model.set_goals(position=trajectory[ptp_index])

            Fb = -1 * Wb
            if stop_on_target_force and np.all(np.abs(Fb)[self.model.target_force != 0] > np.abs(self.model.target_force)[self.model.target_force != 0]):
                self._node.get_logger().info('Target F/T reached {} Stopping!'.format(np.round(Wb, 3)))
                self.set_target_pose(pose=xb, target_time=self.model.dt)
                result = ExecutionResult.STOP_ON_TARGET_FORCE
                break

            if np.any(np.abs(Wb) > max_force_torque):
                self._node.get_logger().error('Maximum force/torque exceeded {}'.format(np.round(Wb, 3)))
                self.set_target_pose(pose=xb, target_time=self.model.dt)
                result = ExecutionResult.FORCE_TORQUE_EXCEEDED
                break

            dxf = self.model.control_position_orientation(Fb, xb)
            xc = transformations.pose_from_angular_velocity(xb, dxf, dt=self.model.dt)
            dt = self.model.dt * (failure_counter + 1)
            result = self._actuate(xc, dt, q_last, reduced_speed)

            if result != ExecutionResult.DONE:
                failure_counter += 1
                if result == ExecutionResult.IK_NOT_FOUND:
                    log[ExecutionResult.IK_NOT_FOUND] += 1
                if result == ExecutionResult.SPEED_LIMIT_EXCEEDED:
                    log[ExecutionResult.SPEED_LIMIT_EXCEEDED] += 1
                continue
            else:
                failure_counter = 0
                q_last = self.joint_angles()

            for _ in range(failure_counter + 1):
                self._rate_sleep()

            standby_time = time.time() - standby_timer
            if standby_time > check_displacement_time:
                displacement_dt = np.linalg.norm(standby_last_pose[:3] - self.end_effector()[:3])
                standby = displacement_dt < displacement_epsilon
                if standby:
                    self._node.get_logger().warn("No more than %s displacement in the last %s seconds" % (round(displacement_dt, 6), check_displacement_time))
                standby_timer = time.time()
                standby_last_pose = self.end_effector()

        if verbose:
            self._node.get_logger().warn("Total # of commands ignored: %s" % log)
        return result

    def _actuate(self, pose, dt, q_last, reduced_speed, attempts=5):
        result = None
        q = self.inverse_kinematics(pose, attempts=0, verbose=False)
        if q is None:
            if attempts > 0:
                return self._actuate(pose, dt, q_last, reduced_speed, attempts - 1)
            self._node.get_logger().warn("IK not found")
            result = ExecutionResult.IK_NOT_FOUND
        else:
            q_speed = (q_last - q) / dt
            if np.any(np.abs(q_speed) > reduced_speed):
                if attempts > 0:
                    return self._actuate(pose, dt, q_last, reduced_speed, attempts - 1)
                self._node.get_logger().warn("Exceeded reduced max speed %s deg/s, Ignoring command" % np.round(np.rad2deg(q_speed), 0))
                result = ExecutionResult.SPEED_LIMIT_EXCEEDED
            else:
                result = self.set_joint_positions(positions=q, target_time=dt)
                self._rate_sleep()
        return result

    def set_hybrid_control(self, model, max_force_torque, timeout=5.0, stop_on_target_force=False):
        reduced_speed = np.deg2rad([100, 100, 100, 150, 150, 150])
        q_last = self.joint_angles()

        initime = time.time()
        xb = self.end_effector()
        failure_counter = 0

        while rclpy.ok() and (time.time() - initime) < timeout:
            Wb = self.get_wrench()
            Fb = -1 * Wb

            if np.any(np.abs(Fb) > max_force_torque):
                self._node.get_logger().error('Maximum force/torque exceeded {}'.format(np.round(Wb, 3)))
                self.set_target_pose(pose=xb, target_time=model.dt)
                return ExecutionResult.FORCE_TORQUE_EXCEEDED

            if stop_on_target_force and np.any(np.abs(Fb)[model.target_force != 0] > model.target_force[model.target_force != 0]):
                self._node.get_logger().info('Target F/T reached {} Stopping!'.format(np.round(Wb, 3)))
                self.set_target_pose(pose=xb, target_time=model.dt)
                return ExecutionResult.STOP_ON_TARGET_FORCE

            xb = self.end_effector()
            dxf = model.control_position_orientation(Fb, xb)
            dxf[:3] = np.clip(dxf[:3], -0.5, 0.5)
            dxf[3:] = np.clip(dxf[3:], -5., 5.)
            xc = transformations.pose_from_angular_velocity(xb, dxf, dt=model.dt)
            dt = model.dt * (failure_counter + 1)

            q = self.inverse_kinematics(xc)
            if q is None:
                self._node.get_logger().warn("IK not found")
                result = ExecutionResult.IK_NOT_FOUND
            else:
                q_speed = (q_last - q) / dt
                if np.any(np.abs(q_speed) > reduced_speed):
                    self._node.get_logger().warn("Exceeded reduced max speed %s deg/s, Ignoring command" % np.round(np.rad2deg(q_speed), 0))
                    result = ExecutionResult.SPEED_LIMIT_EXCEEDED
                else:
                    result = self.set_joint_positions(positions=q, target_time=dt)

            if result != ExecutionResult.DONE:
                failure_counter += 1
                continue
            else:
                failure_counter = 0

            for _ in range(failure_counter + 1):
                self._rate_sleep()

            q_last = self.joint_angles()
        return ExecutionResult.DONE
