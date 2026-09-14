"""Route commitment for the hybrid exploration supervisor.

The supervisor replans only after reaching a short-horizon waypoint, after a
material TARE target change, or when execution has stopped making progress.
This prevents two valid planners from fighting over the robot every timer tick.
"""

from dataclasses import dataclass
import math
from typing import Optional, Sequence, Tuple


Point2 = Tuple[float, float]


@dataclass(frozen=True)
class ActiveRoute:
    source: str
    target: Point2
    path: Tuple[Point2, ...]
    tare_target: Point2
    started_at: float


class RouteCommitment:
    """Own one executable short-horizon route until replanning is justified."""

    def __init__(
        self,
        minimum_duration_s: float,
        maximum_duration_s: float,
        stall_timeout_s: float,
        reached_distance_m: float,
        target_change_distance_m: float,
        progress_distance_m: float,
    ):
        self._minimum_duration_s = minimum_duration_s
        self._maximum_duration_s = maximum_duration_s
        self._stall_timeout_s = stall_timeout_s
        self._reached_distance_m = reached_distance_m
        self._target_change_distance_m = target_change_distance_m
        self._progress_distance_m = progress_distance_m
        self._active: Optional[ActiveRoute] = None
        self._best_distance = float("inf")
        self._last_progress_at = 0.0

    @property
    def active(self) -> Optional[ActiveRoute]:
        return self._active

    def commit(
        self,
        now: float,
        current: Point2,
        tare_target: Point2,
        source: str,
        target: Point2,
        path: Sequence[Point2],
    ) -> ActiveRoute:
        self._active = ActiveRoute(
            source=source,
            target=target,
            path=tuple(path),
            tare_target=tare_target,
            started_at=now,
        )
        self._best_distance = self._distance(current, target)
        self._last_progress_at = now
        return self._active

    def needs_plan(self, now: float, current: Point2, tare_target: Point2) -> bool:
        if self._active is None:
            return True

        distance = self._distance(current, self._active.target)
        if distance <= self._reached_distance_m:
            return True

        if distance <= self._best_distance - self._progress_distance_m:
            self._best_distance = distance
            self._last_progress_at = now

        age = max(0.0, now - self._active.started_at)
        if age < self._minimum_duration_s:
            return False

        target_shift = self._distance(tare_target, self._active.tare_target)
        if target_shift >= self._target_change_distance_m:
            return True
        if now - self._last_progress_at >= self._stall_timeout_s:
            return True
        return age >= self._maximum_duration_s

    @staticmethod
    def _distance(first: Point2, second: Point2) -> float:
        return math.hypot(first[0] - second[0], first[1] - second[1])
