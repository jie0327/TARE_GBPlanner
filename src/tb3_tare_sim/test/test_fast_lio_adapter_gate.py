#!/usr/bin/env python3
"""ROS topic contract for the FAST-LIO-to-TARE startup gate."""

import time
import unittest

import rospy
import rostest
from nav_msgs.msg import Odometry
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool
from std_msgs.msg import Header


class FastLioAdapterGateTest(unittest.TestCase):
    def setUp(self):
        self.state_messages = []
        self.scan_messages = []
        self.start_messages = []
        self.state_sub = rospy.Subscriber(
            "/state_estimation", Odometry, self.state_messages.append, queue_size=10
        )
        self.scan_sub = rospy.Subscriber(
            "/registered_scan", PointCloud2, self.scan_messages.append, queue_size=10
        )
        self.start_sub = rospy.Subscriber(
            "/start_exploration", Bool, self.start_messages.append, queue_size=10
        )
        self.odom_pub = rospy.Publisher("/Odometry", Odometry, queue_size=10)
        self.cloud_pub = rospy.Publisher("/cloud_registered", PointCloud2, queue_size=10)
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            if self.odom_pub.get_num_connections() and self.cloud_pub.get_num_connections():
                return
            time.sleep(0.02)
        self.fail("adapter did not subscribe to FAST-LIO topics")

    def publish_sample(self, stamp):
        odometry = Odometry()
        odometry.header.stamp = stamp
        odometry.header.frame_id = "camera_init"
        odometry.pose.pose.orientation.w = 1.0
        self.odom_pub.publish(odometry)

        header = Header(stamp=stamp, frame_id="camera_init")
        self.cloud_pub.publish(point_cloud2.create_cloud_xyz32(header, [(1.0, 0.0, 0.0)]))
        time.sleep(0.1)

    def test_tare_receives_nothing_until_warmup_is_complete(self):
        self.publish_sample(rospy.Time.from_sec(1.0))
        self.publish_sample(rospy.Time.from_sec(2.0))
        self.assertEqual([], self.state_messages)
        self.assertEqual([], self.scan_messages)
        self.assertEqual([], self.start_messages)

        self.publish_sample(rospy.Time.from_sec(3.0))
        self.assertTrue(self.state_messages)
        self.assertTrue(self.scan_messages)
        self.assertTrue(any(message.data for message in self.start_messages))


if __name__ == "__main__":
    rospy.init_node("fast_lio_adapter_gate_test", anonymous=True)
    rostest.rosrun(
        "tb3_tare_sim",
        "fast_lio_adapter_gate",
        FastLioAdapterGateTest,
    )
