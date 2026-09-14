#!/usr/bin/env python3
"""Publish zero velocity when the internal TB3 command stream goes stale."""

import threading

import rospy
from geometry_msgs.msg import Twist


class TwistWatchdog:
    def __init__(self):
        self._lock = threading.Lock()
        self._last_message_time = rospy.Time(0)
        self._timeout = rospy.Duration(rospy.get_param("~timeout", 0.5))
        input_topic = rospy.get_param("~input_topic", "/tb3/cmd_vel_bridge")
        output_topic = rospy.get_param("~output_topic", "/cmd_vel")
        self._publisher = rospy.Publisher(output_topic, Twist, queue_size=10)
        self._subscriber = rospy.Subscriber(input_topic, Twist, self._callback, queue_size=10)
        self._timer = rospy.Timer(rospy.Duration(0.1), self._watchdog)
        rospy.on_shutdown(self._stop)
        rospy.loginfo("TB3 TARE cmd_vel watchdog: %s -> %s (timeout %.2fs)",
                      input_topic, output_topic, self._timeout.to_sec())

    def _callback(self, message):
        with self._lock:
            self._last_message_time = rospy.Time.now()
        self._publisher.publish(message)

    def _watchdog(self, _event):
        with self._lock:
            timed_out = rospy.Time.now() - self._last_message_time > self._timeout
        if timed_out:
            self._publisher.publish(Twist())

    def _stop(self):
        self._publisher.publish(Twist())


if __name__ == "__main__":
    rospy.init_node("twist_watchdog")
    TwistWatchdog()
    rospy.spin()
