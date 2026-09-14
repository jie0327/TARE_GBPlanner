"""Pure helpers for executing a GBPlanner path through a point-goal controller."""

import math
from typing import Sequence, Tuple


Point2 = Tuple[float, float]
Point3 = Tuple[float, float, float]


def select_path_target(
    current: Point2,
    path: Sequence[Point3],
    previous_index: int,
    lookahead_m: float,
) -> Tuple[Point3, int]:
    """Select a forward-only lookahead point without regressing path progress."""
    if not path:
        raise ValueError("path must contain at least one point")

    start = max(0, min(previous_index, len(path) - 1))
    nearest_index = min(
        range(start, len(path)),
        key=lambda index: math.hypot(
            path[index][0] - current[0], path[index][1] - current[1]
        ),
    )

    distance = math.hypot(
        path[nearest_index][0] - current[0],
        path[nearest_index][1] - current[1],
    )
    if distance >= lookahead_m:
        return path[nearest_index], nearest_index

    for index in range(nearest_index + 1, len(path)):
        distance += math.hypot(
            path[index][0] - path[index - 1][0],
            path[index][1] - path[index - 1][1],
        )
        if distance >= lookahead_m:
            return path[index], index

    return path[-1], len(path) - 1
