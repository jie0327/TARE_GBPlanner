#!/usr/bin/env python3
"""Bridge the official local planner's TwistStamped output to Gazebo Twist."""

import threading

import rospy
from geometry_msgs.msg import Twist, TwistStamped


class TwistBridge:
    def __init__(self):
        self._lock = threading.Lock()
        self._last_message_time = rospy.Time(0)
        self._timeout = rospy.Duration(rospy.get_param("~timeout", 0.5))
        input_topic = rospy.get_param("~input_topic", "/cmd_vel_stamped")
        output_topic = rospy.get_param("~output_topic", "/tb3/cmd_vel_bridge")
        self._publisher = rospy.Publisher(output_topic, Twist, queue_size=10)
        self._subscriber = rospy.Subscriber(input_topic, TwistStamped, self._callback, queue_size=10)
        rospy.Timer(rospy.Duration(0.1), self._watchdog)
        rospy.loginfo("TB3 TARE cmd_vel bridge: %s (TwistStamped) -> %s (Twist)", input_topic, output_topic)

    def _callback(self, message):
        with self._lock:
            self._last_message_time = rospy.Time.now()
        self._publisher.publish(message.twist)

    def _watchdog(self, _event):
        with self._lock:
            timed_out = rospy.Time.now() - self._last_message_time > self._timeout
        if timed_out:
            self._publisher.publish(Twist())


if __name__ == "__main__":
    rospy.init_node("twist_stamped_to_twist")
    TwistBridge()
    rospy.spin()
