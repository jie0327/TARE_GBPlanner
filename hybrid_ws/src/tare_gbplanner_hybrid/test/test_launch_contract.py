#!/usr/bin/env python3
"""Static contracts that keep the proven TARE launch isolated."""

import pathlib
import unittest
import xml.etree.ElementTree as ET


PACKAGE_ROOT = pathlib.Path(__file__).resolve().parents[1]


def find_tare_root():
    candidates = (PACKAGE_ROOT.parents[2], pathlib.Path("/opt/tare_planner"))
    for candidate in candidates:
        if (candidate / "src" / "tb3_tare_sim").is_dir():
            return candidate
    raise RuntimeError("cannot locate the TARE source root")


REPO_ROOT = find_tare_root()
TARE_LAUNCH = REPO_ROOT / "src" / "tb3_tare_sim" / "launch" / "tb3_tare_fast_lio.launch"


class HybridLaunchContractTest(unittest.TestCase):
    def test_original_launch_keeps_original_waypoint_default(self):
        root = ET.parse(TARE_LAUNCH).getroot()
        args = {arg.attrib["name"]: arg.attrib.get("default") for arg in root.findall("arg")}
        self.assertEqual("garage", args["world_name"])
        self.assertEqual(
            "$(find vehicle_simulator)/world/$(arg world_name).world",
            args["world_file"],
        )
        self.assertEqual("/way_point", args["tare_waypoint_topic"])
        self.assertEqual("0.0", args["fast_lio_viz_min_z"])
        self.assertEqual("/start_exploration", args["adapter_start_topic"])
        self.assertEqual(
            "$(find tb3_tare_sim)/config/fast_lio_mapping.rviz",
            args["slam_rviz_config"],
        )

        tare_nodes = [
            node for node in root.findall("node")
            if node.attrib.get("type") == "tare_planner_node"
        ]
        self.assertEqual(1, len(tare_nodes))
        remaps = {item.attrib.get("from"): item.attrib.get("to")
                  for item in tare_nodes[0].findall("remap")}
        self.assertEqual("$(arg tare_waypoint_topic)", remaps["/way_point"])

        adapter_nodes = [
            node for node in root.findall("node")
            if node.attrib.get("type") == "fast_lio_tare_adapter"
        ]
        self.assertEqual(1, len(adapter_nodes))
        adapter_remaps = {item.attrib.get("from"): item.attrib.get("to")
                          for item in adapter_nodes[0].findall("remap")}
        self.assertEqual("$(arg adapter_start_topic)",
                         adapter_remaps["/start_exploration"])

    def test_hybrid_launch_uses_official_gbplanner_and_separate_tare_topic(self):
        launch = PACKAGE_ROOT / "launch" / "tb3_tare_gbplanner_fast_lio_cluttered.launch"
        root = ET.parse(launch).getroot()
        args = {arg.attrib["name"]: arg.attrib.get("default")
                for arg in root.findall("arg")}
        self.assertEqual("0.0", args["fast_lio_viz_min_z"])

        includes = list(root.iter("include"))
        base = [item for item in includes if "tb3_tare_fast_lio.launch" in item.attrib.get("file", "")]
        self.assertEqual(1, len(base))
        forwarded = {arg.attrib.get("name"): arg.attrib.get("value")
                     for arg in base[0].findall("arg")}
        self.assertEqual("garage_cluttered", forwarded["world_name"])
        self.assertEqual("$(arg world_file)", forwarded["world_file"])
        self.assertEqual("$(arg vehicleZ)", forwarded["vehicleZ"])
        self.assertEqual("/hybrid/tare_waypoint", forwarded["tare_waypoint_topic"])
        self.assertEqual("$(arg sensor_profile)", forwarded["sensor_profile"])
        self.assertEqual("$(arg fast_lio_viz_min_z)",
                         forwarded["fast_lio_viz_min_z"])
        self.assertEqual("/hybrid/adapter_start", forwarded["adapter_start_topic"])
        self.assertEqual("$(arg slam_rviz_config)", forwarded["slam_rviz_config"])

        nodes = list(root.iter("node"))
        gbplanner = [node for node in nodes
                     if node.attrib.get("pkg") == "gbplanner"
                     and node.attrib.get("type") == "gbplanner_node"]
        self.assertEqual(1, len(gbplanner))
        supervisor = [node for node in nodes
                      if node.attrib.get("type") == "hybrid_supervisor.py"]
        self.assertEqual(1, len(supervisor))

        gb_remaps = {item.attrib.get("from"): item.attrib.get("to")
                     for item in gbplanner[0].findall("remap")}
        self.assertEqual("/state_estimation", gb_remaps["odometry"])
        self.assertEqual("/velodyne_points_fast_lio", gb_remaps["/pointcloud"])

    def test_pittsburgh_launch_uses_clean_official_mine_geometry(self):
        launch = PACKAGE_ROOT / "launch" / "tb3_tare_gbplanner_fast_lio_pittsburgh.launch"
        root = ET.parse(launch).getroot()

        envs = {item.attrib.get("name"): item.attrib.get("value", "")
                for item in root.findall("env")}
        self.assertIn("GAZEBO_MODEL_PATH", envs)
        self.assertIn("$(arg gbplanner_models)", envs["GAZEBO_MODEL_PATH"])
        args = {arg.attrib["name"]: arg.attrib.get("default")
                for arg in root.findall("arg")}
        self.assertIn("planner_gazebo_sim/models", args["gbplanner_models"])
        self.assertEqual("0.2", args["vehicleZ"])
        self.assertEqual("mid360", args["sensor_profile"])
        self.assertEqual("-5.0", args["fast_lio_viz_min_z"])

        includes = [item for item in root.findall("include")
                    if "tb3_tare_gbplanner_fast_lio_cluttered.launch" in item.attrib.get("file", "")]
        self.assertEqual(1, len(includes))
        forwarded = {arg.attrib.get("name"): arg.attrib.get("value")
                     for arg in includes[0].findall("arg")}
        self.assertEqual(
            "$(find tare_gbplanner_hybrid)/worlds/pittsburgh_mine_clean.world",
            forwarded["world_file"],
        )
        self.assertEqual("tunnel", forwarded["scenario"])
        self.assertEqual("$(arg vehicleZ)", forwarded["vehicleZ"])
        self.assertEqual("$(arg sensor_profile)", forwarded["sensor_profile"])
        self.assertEqual("$(arg fast_lio_viz_min_z)",
                         forwarded["fast_lio_viz_min_z"])
        self.assertEqual("$(arg slam_rviz_config)", forwarded["slam_rviz_config"])
        world = PACKAGE_ROOT / "worlds" / "pittsburgh_mine_clean.world"
        self.assertTrue(world.is_file())
        world_text = world.read_text()
        self.assertIn("model://pittsburgh_mine", world_text)
        self.assertNotIn("librotors_gazebo_ros_interface_plugin.so", world_text)
        self.assertNotIn("<state ", world_text)

    def test_hybrid_docker_is_a_separate_service(self):
        compose_path = REPO_ROOT / "docker-compose.yml"
        if not compose_path.exists():
            self.skipTest("docker-compose.yml is intentionally not copied into the image")
        compose = compose_path.read_text()
        self.assertIn("tare_gbplanner_hybrid:", compose)
        self.assertIn("Dockerfile.hybrid", compose)
        self.assertIn("cmu_tare_official", compose)

    def test_supervisor_searches_from_robot_to_tare_target(self):
        supervisor = (PACKAGE_ROOT / "scripts" / "hybrid_supervisor.py").read_text()
        self.assertIn("planner_search", supervisor)
        self.assertIn("request.use_current_state = True", supervisor)
        self.assertIn("request.target.position.x = tare_target.point.x", supervisor)
        self.assertIn("request.target.position.y = tare_target.point.y", supervisor)
        self.assertIn("planner_srvRequest", supervisor)
        self.assertIn("response.success", supervisor)

    def test_rviz_control_panel_uses_native_cmu_control_topics(self):
        plugin_xml = (PACKAGE_ROOT / "plugin_description.xml").read_text()
        panel_source = (PACKAGE_ROOT / "src" / "exploration_control_panel.cpp").read_text()
        rviz_config = (PACKAGE_ROOT / "config" / "fast_lio_mapping_hybrid.rviz").read_text()

        self.assertIn("tare_gbplanner_hybrid/ExplorationControlPanel", plugin_xml)
        self.assertIn("tare_gbplanner_hybrid/ExplorationControlPanel", rviz_config)
        self.assertIn('advertise<std_msgs::Bool>("/start_exploration"', panel_source)
        self.assertIn('advertise<std_msgs::Int8>("/stop"', panel_source)
        self.assertIn("stop_message.data = 2", panel_source)
        self.assertIn("stop_message.data = 0", panel_source)
        self.assertIn('new QPushButton("Start Navigation"', panel_source)
        self.assertIn('new QPushButton("Stop"', panel_source)
        self.assertIn('new QPushButton("Continue"', panel_source)


if __name__ == "__main__":
    unittest.main()
