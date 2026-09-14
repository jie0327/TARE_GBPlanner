"""Pure policy for selecting TARE-global or GBPlanner-local motion.

The ROS node deliberately delegates graph construction and information gain to
the official GBPlanner2 service.  This module only decides which already
planned candidate should own the next short horizon.
"""

from dataclasses import dataclass
import math
from typing import List, Sequence, Tuple


Point2 = Tuple[float, float]
State2 = Tuple[float, float, float]


@dataclass(frozen=True)
class PolicyConfig:
    narrow_clearance_m: float = 0.7
    wide_clearance_m: float = 2.0
    global_weight_narrow: float = 0.2
    global_weight_open: float = 0.8
    max_single_turn_rad: float = math.radians(89.0)
    turn_penalty: float = 0.18
    cable_penalty: float = 0.45
    cable_soft_limit_rad: float = math.pi
    local_global_alignment_bonus: float = 0.25
    minimum_local_alignment: float = 0.5
    max_local_backtrack_m: float = 0.5
    constrained_target_distance_m: float = 1.5


@dataclass(frozen=True)
class Decision:
    source: str
    target: Point2
    global_weight: float
    local_weight: float
    global_score: float
    local_score: float
    global_admissible: bool
    local_admissible: bool
    clearance_m: float


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def _wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def adaptive_weights(clearance_m: float, config: PolicyConfig) -> Tuple[float, float]:
    span = max(1e-6, config.wide_clearance_m - config.narrow_clearance_m)
    openness = _clamp((clearance_m - config.narrow_clearance_m) / span, 0.0, 1.0)
    global_weight = (
        config.global_weight_narrow
        + openness * (config.global_weight_open - config.global_weight_narrow)
    )
    global_weight = _clamp(global_weight, 0.0, 1.0)
    return global_weight, 1.0 - global_weight


def _segment_headings(current: State2, path: Sequence[Point2]) -> List[float]:
    x, y, yaw = current
    headings: List[float] = []
    previous = (x, y)
    for point in path:
        dx = point[0] - previous[0]
        dy = point[1] - previous[1]
        if math.hypot(dx, dy) > 1e-4:
            headings.append(math.atan2(dy, dx))
            previous = point
    if not headings:
        headings.append(yaw)
    return headings


def _turn_metrics(current: State2, path: Sequence[Point2]) -> Tuple[float, float]:
    headings = _segment_headings(current, path)
    changes = [_wrap(headings[0] - current[2])]
    changes.extend(_wrap(headings[index] - headings[index - 1])
                   for index in range(1, len(headings)))
    return max(abs(change) for change in changes), sum(abs(change) for change in changes)


def _alignment(current: State2, local_target: Point2, global_target: Point2) -> float:
    local = (local_target[0] - current[0], local_target[1] - current[1])
    global_vector = (global_target[0] - current[0], global_target[1] - current[1])
    local_norm = math.hypot(*local)
    global_norm = math.hypot(*global_vector)
    if local_norm < 1e-6 or global_norm < 1e-6:
        return 0.0
    cosine = (local[0] * global_vector[0] + local[1] * global_vector[1]) / (
        local_norm * global_norm
    )
    return 0.5 * (_clamp(cosine, -1.0, 1.0) + 1.0)


def _path_respects_global_progress(
    current: State2,
    path: Sequence[Point2],
    global_target: Point2,
    max_backtrack_m: float,
) -> bool:
    """Reject a local route that gives back substantial global progress."""
    global_dx = global_target[0] - current[0]
    global_dy = global_target[1] - current[1]
    global_norm = math.hypot(global_dx, global_dy)
    if global_norm < 1e-6:
        return True

    unit_x = global_dx / global_norm
    unit_y = global_dy / global_norm
    best_progress = 0.0
    for point in path:
        progress = (
            (point[0] - current[0]) * unit_x
            + (point[1] - current[1]) * unit_y
        )
        if progress < best_progress - max_backtrack_m:
            return False
        best_progress = max(best_progress, progress)
    return True


def _constrained_target(
    current: State2, target: Point2, config: PolicyConfig
) -> Point2:
    """Create a short forward command without a >=90 degree heading jump."""
    dx = target[0] - current[0]
    dy = target[1] - current[1]
    distance = math.hypot(dx, dy)
    if distance < 1e-6:
        return target
    desired_heading = math.atan2(dy, dx)
    heading_change = _clamp(
        _wrap(desired_heading - current[2]),
        -config.max_single_turn_rad,
        config.max_single_turn_rad,
    )
    command_heading = current[2] + heading_change
    command_distance = min(distance, config.constrained_target_distance_m)
    return (
        current[0] + command_distance * math.cos(command_heading),
        current[1] + command_distance * math.sin(command_heading),
    )


def _candidate_penalty(
    current: State2,
    path: Sequence[Point2],
    cable_winding_rad: float,
    config: PolicyConfig,
) -> Tuple[bool, float]:
    max_turn, total_turn = _turn_metrics(current, path)
    admissible = max_turn <= config.max_single_turn_rad
    turn_cost = config.turn_penalty * _clamp(
        total_turn / max(config.max_single_turn_rad, 1e-6), 0.0, 2.0
    )

    first_heading = _segment_headings(current, path)[0]
    first_turn = _wrap(first_heading - current[2])
    current_winding = abs(cable_winding_rad)
    predicted_winding = abs(cable_winding_rad + first_turn)
    added_winding = max(0.0, predicted_winding - current_winding)
    cable_cost = config.cable_penalty * _clamp(
        added_winding / max(config.cable_soft_limit_rad, 1e-6), 0.0, 1.0
    )
    return admissible, turn_cost + cable_cost


def select_target(
    current: State2,
    tare_target: Point2,
    gb_path: Sequence[Point2],
    clearance_m: float,
    cable_winding_rad: float,
    config: PolicyConfig = PolicyConfig(),
) -> Decision:
    """Select one short-horizon candidate through the hybrid policy interface."""
    global_weight, local_weight = adaptive_weights(clearance_m, config)

    global_path = [tare_target]
    global_admissible, global_penalty = _candidate_penalty(
        current, global_path, cable_winding_rad, config
    )
    global_score = global_weight - global_penalty

    local_points = list(gb_path)
    while local_points and math.hypot(
        local_points[0][0] - current[0], local_points[0][1] - current[1]
    ) < 1e-3:
        local_points.pop(0)

    if local_points:
        local_target = local_points[-1]
        local_turn_admissible, local_penalty = _candidate_penalty(
            current, local_points, cable_winding_rad, config
        )
        local_alignment = min(
            _alignment(current, local_points[0], tare_target),
            _alignment(current, local_target, tare_target),
        )
        local_aligned = local_alignment >= config.minimum_local_alignment
        local_admissible = (
            local_turn_admissible
            and local_aligned
            and _path_respects_global_progress(
                current,
                local_points,
                tare_target,
                config.max_local_backtrack_m,
            )
        )
        local_score = (
            local_weight
            + global_weight
            * config.local_global_alignment_bonus
            * local_alignment
            - local_penalty
        )
    else:
        local_target = tare_target
        local_admissible = False
        local_aligned = False
        local_score = float("-inf")

    if not local_points:
        # A high-level TARE waypoint is not an instantaneous steering command:
        # the existing local planner can approach it with a curved path.  When
        # GBPlanner has no candidate, withholding this waypoint would create a
        # permanent hold with no motion from which to acquire more map data.
        source = "tare_global"
        target = (
            tare_target
            if global_admissible
            else _constrained_target(current, tare_target, config)
        )
    elif local_admissible and (not global_admissible or local_score > global_score):
        source = "gbplanner_local"
        target = local_target
    elif global_admissible:
        source = "tare_global"
        target = tare_target
    elif local_aligned:
        # Both candidates exceed the soft turn limit. Stopping at the current
        # pose cannot improve either map coverage or path feasibility and was
        # the source of permanent hold loops. The GBPlanner route is the safer
        # fallback because it is collision checked and leads toward the same
        # TARE target; the downstream local planner applies motion limits.
        source = "gbplanner_local"
        target = _constrained_target(current, local_target, config)
    else:
        # An information-gain route pointing away from TARE is an independent
        # exploration decision, not a local realization of TARE's global
        # objective. Keep moving toward the global objective instead.
        source = "tare_global"
        target = _constrained_target(current, tare_target, config)

    return Decision(
        source=source,
        target=target,
        global_weight=global_weight,
        local_weight=local_weight,
        global_score=global_score,
        local_score=local_score,
        global_admissible=global_admissible,
        local_admissible=local_admissible,
        clearance_m=clearance_m,
    )
