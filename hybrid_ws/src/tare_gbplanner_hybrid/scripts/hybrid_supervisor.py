#!/usr/bin/env python3
"""Adaptive TARE-global / GBPlanner2-local exploration supervisor."""

import math
import threading
import time

import rospy
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Odometry, Path
from planner_msgs.srv import (
    planner_search,
    planner_searchRequest,
    planner_srv,
    planner_srvRequest,
)
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32, Float32MultiArray, String
from tf.transformations import euler_from_quaternion

from tare_gbplanner_hybrid.commitment import RouteCommitment
from tare_gbplanner_hybrid.path_tracking import select_path_target
from tare_gbplanner_hybrid.policy import PolicyConfig, select_target


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class HybridSupervisor:
    def __init__(self):
        self._lock = threading.Lock()
        self._planning = False
        self._state = None
        self._tare_target = None
        self._clearance = rospy.get_param("~wide_clearance_m", 2.0)
        self._last_cloud_wall_time = 0.0
        self._last_yaw = None
        self._cable_winding = 0.0

        self._map_frame = rospy.get_param("~map_frame", "map")
        self._planner_service_name = rospy.get_param(
            "~gbplanner_search_service", "/gbplanner/search"
        )
        self._exploration_service_name = rospy.get_param(
            "~gbplanner_exploration_service", "/gbplanner"
        )
        self._planning_period = rospy.get_param("~planning_period", 1.0)
        self._cloud_period = rospy.get_param("~clearance_update_period", 0.5)
        self._scan_radius = rospy.get_param("~clearance_scan_radius", 3.0)
        self._scan_percentile = rospy.get_param("~clearance_percentile", 0.05)
        self._obstacle_min_z = rospy.get_param("~obstacle_min_z_relative", -0.08)
        self._obstacle_max_z = rospy.get_param("~obstacle_max_z_relative", 0.45)
        self._gb_lookahead = rospy.get_param("~gbplanner_lookahead_m", 2.5)
        self._tracking_lookahead = rospy.get_param(
            "~gbplanner_tracking_lookahead_m", 0.8
        )
        self._route_commitment = RouteCommitment(
            minimum_duration_s=rospy.get_param("~minimum_route_duration", 3.0),
            maximum_duration_s=rospy.get_param("~maximum_route_duration", 10.0),
            stall_timeout_s=rospy.get_param("~route_stall_timeout", 4.0),
            reached_distance_m=rospy.get_param("~route_reached_distance", 0.45),
            target_change_distance_m=rospy.get_param(
                "~tare_target_change_distance", 2.0
            ),
            progress_distance_m=rospy.get_param("~route_progress_distance", 0.10),
        )
        self._committed_local_poses = []
        self._committed_path_index = 0
        self._committed_target_z = 0.0

        self._policy = PolicyConfig(
            narrow_clearance_m=rospy.get_param("~narrow_clearance_m", 0.7),
            wide_clearance_m=rospy.get_param("~wide_clearance_m", 2.0),
            global_weight_narrow=rospy.get_param("~global_weight_narrow", 0.2),
            global_weight_open=rospy.get_param("~global_weight_open", 0.8),
            max_single_turn_rad=math.radians(
                rospy.get_param("~max_single_turn_deg", 89.0)
            ),
            turn_penalty=rospy.get_param("~turn_penalty", 0.18),
            cable_penalty=rospy.get_param("~cable_penalty", 0.45),
            cable_soft_limit_rad=math.radians(
                rospy.get_param("~cable_soft_limit_deg", 180.0)
            ),
            local_global_alignment_bonus=rospy.get_param(
                "~local_global_alignment_bonus", 0.25
            ),
            minimum_local_alignment=rospy.get_param(
                "~minimum_local_alignment", 0.5
            ),
            max_local_backtrack_m=rospy.get_param(
                "~max_local_backtrack_m", 0.5
            ),
            constrained_target_distance_m=rospy.get_param(
                "~constrained_target_distance", 1.5
            ),
        )

        self._waypoint_pub = rospy.Publisher("/way_point", PointStamped, queue_size=2)
        self._path_pub = rospy.Publisher("/hybrid/selected_path", Path, queue_size=1)
        self._mode_pub = rospy.Publisher(
            "/hybrid/mode", String, queue_size=1, latch=True
        )
        self._diagnostics_pub = rospy.Publisher(
            "/hybrid/weights", Float32MultiArray, queue_size=1
        )

        rospy.Subscriber(
            "/hybrid/tare_waypoint", PointStamped, self._tare_waypoint_callback,
            queue_size=2,
        )
        rospy.Subscriber(
            "/state_estimation", Odometry, self._odometry_callback, queue_size=20
        )
        rospy.Subscriber(
            "/registered_scan", PointCloud2, self._cloud_callback, queue_size=1
        )
        rospy.Subscriber(
            "/cable/winding_angle", Float32, self._cable_callback, queue_size=1
        )

        self._gbplanner = rospy.ServiceProxy(
            self._planner_service_name, planner_search, persistent=False
        )
        self._gbplanner_exploration = rospy.ServiceProxy(
            self._exploration_service_name, planner_srv, persistent=False
        )
        self._timer = rospy.Timer(
            rospy.Duration(max(0.1, self._planning_period)), self._plan_callback
        )
        rospy.loginfo(
            "Hybrid supervisor ready: TARE global target + official GBPlanner2 local path"
        )

    def _tare_waypoint_callback(self, message):
        with self._lock:
            self._tare_target = message

    def _odometry_callback(self, message):
        quaternion = message.pose.pose.orientation
        yaw = euler_from_quaternion(
            [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        )[2]
        with self._lock:
            if self._last_yaw is not None:
                self._cable_winding += wrap_angle(yaw - self._last_yaw)
            self._last_yaw = yaw
            self._state = (message.pose.pose.position.x,
                           message.pose.pose.position.y,
                           message.pose.pose.position.z,
                           yaw)

    def _cable_callback(self, message):
        # A real tether sensor can override the odometry-derived winding proxy.
        with self._lock:
            self._cable_winding = message.data

    def _cloud_callback(self, message):
        now = time.monotonic()
        if now - self._last_cloud_wall_time < self._cloud_period:
            return
        with self._lock:
            state = self._state
        if state is None:
            return

        distances = []
        for x, y, z in point_cloud2.read_points(
            message, field_names=("x", "y", "z"), skip_nans=True
        ):
            relative_z = z - state[2]
            if relative_z < self._obstacle_min_z or relative_z > self._obstacle_max_z:
                continue
            distance = math.hypot(x - state[0], y - state[1])
            if 0.15 < distance <= self._scan_radius:
                distances.append(distance)

        if distances:
            distances.sort()
            index = min(
                len(distances) - 1,
                max(0, int(self._scan_percentile * (len(distances) - 1))),
            )
            clearance = distances[index]
        else:
            clearance = self._scan_radius

        with self._lock:
            self._clearance = clearance
            self._last_cloud_wall_time = now

    def _request_gbplanner_path(self, tare_target):
        request = planner_searchRequest()
        request.header.stamp = rospy.Time.now()
        request.header.frame_id = self._map_frame
        request.use_current_state = True
        request.target.position.x = tare_target.point.x
        request.target.position.y = tare_target.point.y
        request.target.position.z = tare_target.point.z
        request.target.orientation.w = 1.0
        request.bound_mode = request.kExtendedBound
        try:
            rospy.wait_for_service(self._planner_service_name, timeout=0.05)
            response = self._gbplanner(request)
            if response.success:
                return response.path
            rospy.loginfo_throttle(
                10.0,
                "TARE target is outside GBPlanner's known map; using "
                "direction-filtered local information gain",
            )
            return self._request_gbplanner_exploration_path()
        except (rospy.ROSException, rospy.ServiceException) as error:
            rospy.logwarn_throttle(
                10.0,
                "GBPlanner2 target search unavailable; trying local exploration: %s"
                % error,
            )
            return self._request_gbplanner_exploration_path()

    def _request_gbplanner_exploration_path(self):
        request = planner_srvRequest()
        request.header.stamp = rospy.Time.now()
        request.header.frame_id = self._map_frame
        request.bound_mode = request.kExtendedBound
        request.root_pose.orientation.w = 1.0
        try:
            rospy.wait_for_service(self._exploration_service_name, timeout=0.05)
            response = self._gbplanner_exploration(request)
            return response.path
        except (rospy.ROSException, rospy.ServiceException) as error:
            rospy.logwarn_throttle(
                10.0,
                "GBPlanner2 unavailable; passing through TARE target: %s" % error,
            )
            return []

    def _truncate_path(self, poses, state):
        if not poses:
            return []
        output = []
        previous = (state[0], state[1])
        distance = 0.0
        for pose in poses:
            point = (pose.position.x, pose.position.y)
            if math.hypot(point[0] - previous[0], point[1] - previous[1]) < 1e-3:
                continue
            distance += math.hypot(point[0] - previous[0], point[1] - previous[1])
            output.append(pose)
            previous = point
            if distance >= self._gb_lookahead:
                break
        return output

    def _plan_callback(self, _event):
        with self._lock:
            if self._planning:
                return
            state = self._state
            tare_target = self._tare_target
            clearance = self._clearance
            cable_winding = self._cable_winding
            self._planning = True

        try:
            if tare_target is None:
                return
            if state is None:
                self._waypoint_pub.publish(tare_target)
                return

            tare_xy = (tare_target.point.x, tare_target.point.y)
            now = rospy.get_time()
            if not self._route_commitment.needs_plan(
                now, (state[0], state[1]), tare_xy
            ):
                active = self._route_commitment.active
                self._publish_committed_route(state, active)
                return

            local_poses = self._truncate_path(
                self._request_gbplanner_path(tare_target), state
            )
            local_xy = [(pose.position.x, pose.position.y) for pose in local_poses]
            decision = select_target(
                current=(state[0], state[1], state[3]),
                tare_target=tare_xy,
                gb_path=local_xy,
                clearance_m=clearance,
                cable_winding_rad=cable_winding,
                config=self._policy,
            )

            committed_path = local_xy if decision.source == "gbplanner_local" else []
            active = self._route_commitment.commit(
                now=now,
                current=(state[0], state[1]),
                tare_target=tare_xy,
                source=decision.source,
                target=decision.target,
                path=committed_path,
            )
            self._committed_local_poses = (
                list(local_poses) if decision.source == "gbplanner_local" else []
            )
            self._committed_path_index = 0
            self._committed_target_z = (
                local_poses[-1].position.z
                if decision.source == "gbplanner_local" and local_poses
                else tare_target.point.z
            )

            self._publish_committed_route(state, active)
            diagnostics = Float32MultiArray()
            diagnostics.data = [
                decision.global_weight,
                decision.local_weight,
                decision.clearance_m,
                decision.global_score,
                decision.local_score if math.isfinite(decision.local_score) else -1.0,
                cable_winding,
            ]
            self._diagnostics_pub.publish(diagnostics)
            rospy.loginfo_throttle(
                3.0,
                "hybrid=%s clearance=%.2f weights(TARE=%.2f, GB=%.2f)"
                % (decision.source, decision.clearance_m,
                   decision.global_weight, decision.local_weight),
            )
        finally:
            with self._lock:
                self._planning = False

    def _publish_committed_route(self, state, active):
        waypoint = PointStamped()
        waypoint.header.stamp = rospy.Time.now()
        waypoint.header.frame_id = self._map_frame
        if active.source == "gbplanner_local" and self._committed_local_poses:
            path = [
                (pose.position.x, pose.position.y, pose.position.z)
                for pose in self._committed_local_poses
            ]
            tracking_target, self._committed_path_index = select_path_target(
                current=(state[0], state[1]),
                path=path,
                previous_index=self._committed_path_index,
                lookahead_m=self._tracking_lookahead,
            )
            waypoint.point.x = tracking_target[0]
            waypoint.point.y = tracking_target[1]
            waypoint.point.z = tracking_target[2]
        else:
            waypoint.point.x, waypoint.point.y = active.target
            waypoint.point.z = self._committed_target_z
        self._waypoint_pub.publish(waypoint)
        self._publish_path(
            active.source, state, waypoint, self._committed_local_poses
        )
        self._mode_pub.publish(String(data=active.source))

    def _publish_path(self, source, state, waypoint, local_poses):
        path = Path()
        path.header = waypoint.header
        current = PoseStamped()
        current.header = waypoint.header
        current.pose.position.x = state[0]
        current.pose.position.y = state[1]
        current.pose.position.z = state[2]
        current.pose.orientation.w = 1.0
        path.poses.append(current)

        if source == "gbplanner_local":
            for pose in local_poses:
                stamped = PoseStamped()
                stamped.header = waypoint.header
                stamped.pose = pose
                path.poses.append(stamped)
        else:
            target = PoseStamped()
            target.header = waypoint.header
            target.pose.position = waypoint.point
            target.pose.orientation.w = 1.0
            path.poses.append(target)
        self._path_pub.publish(path)


if __name__ == "__main__":
    rospy.init_node("tare_gbplanner_hybrid_supervisor")
    HybridSupervisor()
    rospy.spin()
