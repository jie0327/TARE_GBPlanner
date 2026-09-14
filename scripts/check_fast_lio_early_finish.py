#!/usr/bin/env python3
"""Fail when TARE reports completion before the robot has explored far enough."""

import math
import sys
import time

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool


class EarlyFinishCheck:
    def __init__(self):
        self.finish = False
        self.first_truth = None
        self.last_truth = None
        self.latest_truth_yaw = None
        self.first_truth_yaw = None
        self.truth_distance = 0.0
        self.fast_position = None
        self.first_fast = None
        self.last_fast = None
        self.fast_distance = 0.0
        self.fast_z_min = math.inf
        self.fast_z_max = -math.inf
        self.max_abs_roll = 0.0
        self.max_abs_pitch = 0.0
        self.last_state_stamp = None
        self.last_cloud_stamp = None
        self.max_stamp_delta = 0.0
        self.first_fast_yaw = None
        self.max_relative_xy_error = 0.0
        self.max_relative_yaw_error = 0.0
        self.max_uncovered = 0
        self.last_uncovered = 0

        rospy.Subscriber("/odom", Odometry, self._truth_callback, queue_size=20)
        rospy.Subscriber("/state_estimation", Odometry, self._fast_callback, queue_size=20)
        rospy.Subscriber("/registered_scan", PointCloud2, self._cloud_callback, queue_size=20)
        rospy.Subscriber(
            "/sensor_coverage_planner/uncovered_cloud",
            PointCloud2,
            self._uncovered_callback,
            queue_size=5,
        )
        rospy.Subscriber(
            "/sensor_coverage_planner/exploration_finish",
            Bool,
            self._finish_callback,
            queue_size=5,
        )

    def _truth_callback(self, message):
        position = message.pose.pose.position
        current = (position.x, position.y)
        if self.first_truth is None:
            self.first_truth = current
            self.first_truth_yaw = self._yaw(message.pose.pose.orientation)
        if self.last_truth is not None:
            self.truth_distance += math.hypot(
                current[0] - self.last_truth[0], current[1] - self.last_truth[1]
            )
        self.last_truth = current
        self.latest_truth_yaw = self._yaw(message.pose.pose.orientation)

    def _fast_callback(self, message):
        position = message.pose.pose.position
        self.fast_position = (position.x, position.y, position.z)
        if self.first_fast is None:
            self.first_fast = self.fast_position
            self.first_fast_yaw = self._yaw(message.pose.pose.orientation)
        if self.last_fast is not None:
            self.fast_distance += math.sqrt(
                (position.x - self.last_fast[0]) ** 2
                + (position.y - self.last_fast[1]) ** 2
                + (position.z - self.last_fast[2]) ** 2
            )
        self.last_fast = self.fast_position
        self.fast_z_min = min(self.fast_z_min, position.z)
        self.fast_z_max = max(self.fast_z_max, position.z)
        q = message.pose.pose.orientation
        sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        sinp = max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x)))
        pitch = math.asin(sinp)
        self.max_abs_roll = max(self.max_abs_roll, abs(roll))
        self.max_abs_pitch = max(self.max_abs_pitch, abs(pitch))
        self.last_state_stamp = message.header.stamp.to_sec()
        self._update_stamp_delta()
        self._update_relative_error(message.pose.pose.orientation)

    @staticmethod
    def _yaw(q):
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    @staticmethod
    def _wrap(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def _update_relative_error(self, fast_orientation):
        if self.first_truth is None or self.first_fast is None or self.last_truth is None:
            return
        truth_dx = self.last_truth[0] - self.first_truth[0]
        truth_dy = self.last_truth[1] - self.first_truth[1]
        truth_c = math.cos(-self.first_truth_yaw)
        truth_s = math.sin(-self.first_truth_yaw)
        truth_relative = (
            truth_c * truth_dx - truth_s * truth_dy,
            truth_s * truth_dx + truth_c * truth_dy,
        )
        fast_dx = self.fast_position[0] - self.first_fast[0]
        fast_dy = self.fast_position[1] - self.first_fast[1]
        fast_c = math.cos(-self.first_fast_yaw)
        fast_s = math.sin(-self.first_fast_yaw)
        fast_relative = (
            fast_c * fast_dx - fast_s * fast_dy,
            fast_s * fast_dx + fast_c * fast_dy,
        )
        self.max_relative_xy_error = max(
            self.max_relative_xy_error,
            math.hypot(
                fast_relative[0] - truth_relative[0],
                fast_relative[1] - truth_relative[1],
            ),
        )
        relative_truth_yaw = self._wrap(self.latest_truth_yaw - self.first_truth_yaw)
        relative_fast_yaw = self._wrap(self._yaw(fast_orientation) - self.first_fast_yaw)
        self.max_relative_yaw_error = max(
            self.max_relative_yaw_error,
            abs(self._wrap(relative_fast_yaw - relative_truth_yaw)),
        )

    def _uncovered_callback(self, message):
        point_count = message.width * message.height
        self.last_uncovered = point_count
        self.max_uncovered = max(self.max_uncovered, point_count)

    def _cloud_callback(self, message):
        self.last_cloud_stamp = message.header.stamp.to_sec()
        self._update_stamp_delta()

    def _update_stamp_delta(self):
        if self.last_state_stamp is not None and self.last_cloud_stamp is not None:
            self.max_stamp_delta = max(
                self.max_stamp_delta,
                abs(self.last_state_stamp - self.last_cloud_stamp),
            )

    def _finish_callback(self, message):
        self.finish = message.data


def main():
    rospy.init_node("fast_lio_early_finish_check", anonymous=True)
    duration = float(rospy.get_param("~duration", 25.0))
    minimum_distance = float(rospy.get_param("~minimum_distance", 10.0))
    check = EarlyFinishCheck()
    deadline = time.monotonic() + duration

    while not rospy.is_shutdown() and time.monotonic() < deadline and not check.finish:
        time.sleep(0.05)

    print(
        "finish={} truth_distance={:.3f} fast_distance={:.3f} fast_position={} "
        "fast_z_range=({:.3f},{:.3f}) max_roll_deg={:.2f} max_pitch_deg={:.2f} "
        "max_relative_xy_error={:.3f} max_relative_yaw_error_deg={:.2f} "
        "max_stamp_delta={:.4f} uncovered_last={} uncovered_max={}".format(
            check.finish,
            check.truth_distance,
            check.fast_distance,
            check.fast_position,
            check.fast_z_min,
            check.fast_z_max,
            math.degrees(check.max_abs_roll),
            math.degrees(check.max_abs_pitch),
            check.max_relative_xy_error,
            math.degrees(check.max_relative_yaw_error),
            check.max_stamp_delta,
            check.last_uncovered,
            check.max_uncovered,
        )
    )
    if check.finish and check.truth_distance < minimum_distance:
        print("FAIL: TARE finished before the minimum exploration distance")
        return 1
    print("PASS: no premature exploration finish observed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
