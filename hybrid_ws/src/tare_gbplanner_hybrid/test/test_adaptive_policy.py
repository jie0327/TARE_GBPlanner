#!/usr/bin/env python3
"""Behavioral tests for the hybrid planner's policy seam."""

import math
import pathlib
import sys
import unittest


PACKAGE_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT / "src"))

from tare_gbplanner_hybrid.commitment import RouteCommitment  # noqa: E402
from tare_gbplanner_hybrid.policy import PolicyConfig, select_target  # noqa: E402


class AdaptivePolicyTest(unittest.TestCase):
    def setUp(self):
        self.config = PolicyConfig(
            narrow_clearance_m=0.7,
            wide_clearance_m=2.0,
            global_weight_narrow=0.2,
            global_weight_open=0.8,
            max_single_turn_rad=math.radians(80.0),
        )

    def test_open_space_prefers_tare_global_coverage(self):
        decision = select_target(
            current=(0.0, 0.0, 0.0),
            tare_target=(4.0, 0.0),
            gb_path=[(0.0, 0.0), (0.0, 2.0)],
            clearance_m=2.5,
            cable_winding_rad=0.0,
            config=self.config,
        )

        self.assertEqual("tare_global", decision.source)
        self.assertAlmostEqual(0.8, decision.global_weight)
        self.assertAlmostEqual(0.2, decision.local_weight)

    def test_narrow_space_prefers_gbplanner_local_graph(self):
        decision = select_target(
            current=(0.0, 0.0, 0.0),
            tare_target=(4.0, 0.0),
            gb_path=[(0.0, 0.0), (1.0, 0.1), (2.0, 0.2)],
            clearance_m=0.4,
            cable_winding_rad=0.0,
            config=self.config,
        )

        self.assertEqual("gbplanner_local", decision.source)
        self.assertEqual((2.0, 0.2), decision.target)
        self.assertAlmostEqual(0.2, decision.global_weight)
        self.assertAlmostEqual(0.8, decision.local_weight)

    def test_turn_over_limit_is_rejected_even_in_narrow_space(self):
        decision = select_target(
            current=(0.0, 0.0, 0.0),
            tare_target=(3.0, 0.0),
            gb_path=[(0.0, 0.0), (-1.0, 1.0)],
            clearance_m=0.3,
            cable_winding_rad=0.0,
            config=self.config,
        )

        self.assertEqual("tare_global", decision.source)
        self.assertFalse(decision.local_admissible)

    def test_cable_winding_penalty_prefers_unwinding_path(self):
        decision = select_target(
            current=(0.0, 0.0, 0.0),
            tare_target=(1.0, 1.0),
            gb_path=[(0.0, 0.0), (1.0, -1.0)],
            clearance_m=1.0,
            cable_winding_rad=1.5,
            config=self.config,
        )

        self.assertEqual("gbplanner_local", decision.source)

    def test_missing_gbplanner_path_falls_back_to_tare(self):
        decision = select_target(
            current=(1.0, 2.0, 0.2),
            tare_target=(3.0, 4.0),
            gb_path=[],
            clearance_m=0.2,
            cable_winding_rad=0.0,
            config=self.config,
        )

        self.assertEqual("tare_global", decision.source)
        self.assertEqual((3.0, 4.0), decision.target)

    def test_missing_local_path_never_deadlocks_on_global_bearing(self):
        decision = select_target(
            current=(0.0, 0.0, 0.0),
            tare_target=(-1.0, 4.0),
            gb_path=[],
            clearance_m=0.2,
            cable_winding_rad=0.0,
            config=self.config,
        )

        self.assertEqual("tare_global", decision.source)
        heading = math.atan2(decision.target[1], decision.target[0])
        self.assertLessEqual(abs(heading), self.config.max_single_turn_rad + 1e-6)
        self.assertLessEqual(math.hypot(*decision.target), 1.5 + 1e-6)

    def test_both_turn_penalties_fall_back_to_motion_instead_of_hold(self):
        decision = select_target(
            current=(0.0, 0.0, 0.0),
            tare_target=(-4.0, 0.0),
            gb_path=[(-1.0, 0.0), (-2.0, 0.0)],
            clearance_m=0.3,
            cable_winding_rad=0.0,
            config=self.config,
        )

        self.assertNotEqual("hold", decision.source)
        heading = math.atan2(decision.target[1], decision.target[0])
        self.assertLessEqual(abs(heading), self.config.max_single_turn_rad + 1e-6)

    def test_gbplanner_candidate_opposite_to_tare_never_wins(self):
        decision = select_target(
            current=(0.0, 0.0, math.pi),
            tare_target=(4.0, 0.0),
            gb_path=[(-1.0, 0.0), (-2.0, 0.0)],
            clearance_m=0.3,
            cable_winding_rad=0.0,
            config=self.config,
        )

        self.assertEqual("tare_global", decision.source)
        self.assertFalse(decision.local_admissible)

    def test_gbplanner_path_that_starts_opposite_to_tare_is_rejected(self):
        decision = select_target(
            current=(0.0, 0.0, math.pi),
            tare_target=(4.0, 0.0),
            gb_path=[
                (-1.0, 0.0),
                (-1.5, 0.866),
                (-1.5, 1.866),
                (-1.0, 2.732),
                (0.5, 2.732),
            ],
            clearance_m=0.3,
            cable_winding_rad=0.0,
            config=self.config,
        )

        self.assertEqual("tare_global", decision.source)
        self.assertFalse(decision.local_admissible)
        command_heading = math.atan2(decision.target[1], decision.target[0])
        self.assertLessEqual(
            abs(math.atan2(math.sin(command_heading - math.pi),
                           math.cos(command_heading - math.pi))),
            self.config.max_single_turn_rad + 1e-6,
        )


class RouteCommitmentTest(unittest.TestCase):
    def setUp(self):
        self.commitment = RouteCommitment(
            minimum_duration_s=3.0,
            maximum_duration_s=10.0,
            stall_timeout_s=4.0,
            reached_distance_m=0.4,
            target_change_distance_m=2.0,
            progress_distance_m=0.1,
        )
        self.commitment.commit(
            now=0.0,
            current=(0.0, 0.0),
            tare_target=(8.0, 0.0),
            source="gbplanner_local",
            target=(2.5, 0.0),
            path=[(1.0, 0.0), (2.5, 0.0)],
        )

    def test_route_is_kept_while_progressing(self):
        self.assertFalse(
            self.commitment.needs_plan(
                now=1.0, current=(0.3, 0.0), tare_target=(8.5, 0.0)
            )
        )
        self.assertFalse(
            self.commitment.needs_plan(
                now=4.0, current=(1.0, 0.0), tare_target=(8.5, 0.0)
            )
        )
        self.assertEqual((2.5, 0.0), self.commitment.active.target)

    def test_reaching_committed_waypoint_requests_a_new_plan(self):
        self.assertTrue(
            self.commitment.needs_plan(
                now=2.0, current=(2.2, 0.0), tare_target=(8.0, 0.0)
            )
        )

    def test_stalled_route_requests_a_new_plan_after_minimum_duration(self):
        self.assertFalse(
            self.commitment.needs_plan(
                now=2.0, current=(0.0, 0.0), tare_target=(8.0, 0.0)
            )
        )
        self.assertTrue(
            self.commitment.needs_plan(
                now=4.1, current=(0.0, 0.0), tare_target=(8.0, 0.0)
            )
        )


if __name__ == "__main__":
    unittest.main()
