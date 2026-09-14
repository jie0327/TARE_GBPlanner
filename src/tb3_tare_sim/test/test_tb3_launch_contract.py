#!/usr/bin/env python3
"""Static contract checks for the standalone TB3/TARE launch package."""

import pathlib
import unittest
import xml.etree.ElementTree as ET


PACKAGE_ROOT = pathlib.Path(__file__).resolve().parents[1]


class Tb3LaunchContractTest(unittest.TestCase):
    def test_package_and_launch_exist(self):
        self.assertTrue((PACKAGE_ROOT / "package.xml").is_file())
        launch = PACKAGE_ROOT / "launch" / "tb3_tare.launch"
        self.assertTrue(launch.is_file())
        ET.parse(launch)

    def test_launch_keeps_original_stack_and_uses_tb3_adapters(self):
        root = ET.parse(PACKAGE_ROOT / "launch" / "tb3_tare.launch").getroot()
        includes = {node.attrib.get("file", "") for node in root.iter("include")}
        self.assertTrue(any("terrain_analysis.launch" in item for item in includes))
        self.assertTrue(any("local_planner.launch" in item for item in includes))
        types = {node.attrib.get("type", "") for node in root.findall("node")}
        self.assertIn("twist_stamped_to_twist.py", types)
        self.assertIn("twist_watchdog.py", types)
        self.assertIn("tb3_state_scan_adapter", types)
        self.assertTrue(any(node.attrib.get("if") == "$(arg joystick)"
                            for node in root.findall("group")))
        rviz_nodes = [node for node in root.findall("node")
                      if node.attrib.get("type") == "rviz"]
        self.assertEqual(len(rviz_nodes), 1)
        self.assertIn("vehicle_simulator)/rviz/vehicle_simulator.rviz",
                      rviz_nodes[0].attrib.get("args", ""))

    def test_urdf_has_diff_drive_and_profile_argument(self):
        urdf = PACKAGE_ROOT / "urdf" / "turtlebot3_waffle_tare.urdf.xacro"
        self.assertTrue(urdf.is_file())
        text = urdf.read_text()
        self.assertIn('xacro:arg name="sensor_profile"', text)
        self.assertIn('libgazebo_ros_diff_drive.so', text)
        self.assertIn('$(find velodyne_description)/urdf/VLP-16.urdf.xacro', text)

    def test_mid360_profile_keeps_the_controlled_velodyne_comparison(self):
        """The Mid-360 profile remains the lightweight VLP-based comparison."""
        urdf = PACKAGE_ROOT / "urdf" / "turtlebot3_waffle_tare.urdf.xacro"
        text = urdf.read_text()
        self.assertIn('xacro:VLP-16 parent="${namespace}base_link"', text)
        self.assertIn('topic="/velodyne_points" hz="10"', text)
        self.assertIn('samples="180" lasers="16"', text)
        self.assertIn('min_range="0.1" max_range="70.0"', text)

    def test_fast_lio_launch_uses_slam_outputs_instead_of_gazebo_truth(self):
        launch = PACKAGE_ROOT / "launch" / "tb3_tare_fast_lio.launch"
        self.assertTrue(launch.is_file())
        root = ET.parse(launch).getroot()
        args = {arg.attrib["name"]: arg.attrib.get("default")
                for arg in root.findall("arg")}
        self.assertEqual("false", args["rviz"])
        self.assertEqual("true", args["slam_rviz"])
        self.assertEqual("mid360", args["sensor_profile"])

        types = {node.attrib.get("type", "") for node in root.iter("node")}
        self.assertIn("fastlio_mapping", types)
        self.assertIn("fast_lio_tare_adapter", types)
        self.assertNotIn("tb3_state_scan_adapter", types)

        text = launch.read_text()
        self.assertIn("fast_lio_sim.yaml", text)
        self.assertIn("sensor_profile:=$(arg sensor_profile)", text)
        self.assertIn('to="/velodyne_points"', text)
        self.assertIn('-z $(arg vehicleZ)', text)
        self.assertIn("publish_odom_tf:=false", text)
        self.assertIn('name="kAutoStart" type="bool" value="false"', text)
        self.assertIn('name="fast_lio_warmup_cloud_count" default="30"', text)
        self.assertIn('name="fast_lio_viz_rate" default="1.0"', text)
        self.assertIn('name="fast_lio_viz_voxel_size" default="0.10"', text)
        self.assertIn('name="fast_lio_viz_min_z" default="0.0"', text)
        self.assertIn('name="publish/map_publish_en" type="bool" value="true"', text)
        self.assertIn('name="publish/map_publish_interval" type="int" value="10"', text)
        self.assertIn(
            'name="ready_cloud_count" value="$(arg fast_lio_warmup_cloud_count)"',
            text,
        )
        self.assertIn('name="lidar_offset_z" value="0.054"', text)

        config = (PACKAGE_ROOT / "config" / "fast_lio_sim.yaml").read_text()
        self.assertIn('lid_topic: "/velodyne_points_fast_lio"', config)
        self.assertIn('imu_topic: "/imu/data"', config)
        self.assertIn("lidar_type: 4", config)
        self.assertIn("blind: 0.2", config)
        self.assertIn("det_range: 70.0", config)
        self.assertIn("extrinsic_T: [-0.064, 0.0, 0.054]", config)

        nodelets = [node for node in root.iter("node") if node.attrib.get("pkg") == "nodelet"]
        self.assertTrue(any("pcl/PassThrough" in node.attrib.get("args", "") for node in nodelets))
        self.assertTrue(any("pcl/VoxelGrid" in node.attrib.get("args", "") for node in nodelets))
        ground_filters = [node for node in nodelets
                          if node.attrib.get("name") == "fast_lio_viz_ground_filter"]
        self.assertEqual(1, len(ground_filters))
        self.assertEqual("pcl/PassThrough", ground_filters[0].attrib.get("args", "").split()[-1])
        ground_filter_text = ET.tostring(ground_filters[0], encoding="unicode")
        self.assertIn('name="filter_field_name" value="z"', ground_filter_text)
        self.assertIn('name="filter_limit_min" value="$(arg fast_lio_viz_min_z)"',
                      ground_filter_text)
        self.assertTrue(any(node.attrib.get("pkg") == "topic_tools" and
                            node.attrib.get("type") == "throttle"
                            for node in root.iter("node")))
        throttle_nodes = [node for node in root.iter("node")
                          if node.attrib.get("name") == "fast_lio_viz_throttle"]
        self.assertEqual(1, len(throttle_nodes))
        self.assertIn("messages /Laser_map ", throttle_nodes[0].attrib.get("args", ""))

        rviz_nodes = [node for node in root.findall("node")
                      if node.attrib.get("type") == "rviz"]
        default_visible = [node for node in rviz_nodes
                           if node.attrib.get("if") == "$(arg slam_rviz)"]
        self.assertEqual(1, len(default_visible))
        self.assertIn("tb3_tare_sim)/config/fast_lio_mapping.rviz",
                      default_visible[0].attrib.get("args", ""))

        rviz = (PACKAGE_ROOT / "config" / "fast_lio_mapping.rviz").read_text()
        self.assertIn("Topic: /cloud_registered_viz", rviz)
        self.assertNotIn("Decay Time: 300", rviz)
        self.assertIn("Queue Size: 1", rviz)
        self.assertIn("- Alpha: 0.4", rviz)
        self.assertIn("Size (Pixels): 2", rviz)

        fast_lio_source = (PACKAGE_ROOT.parent / "FAST_LIO" / "src" /
                           "laserMapping.cpp").read_text()
        self.assertIn('nh.param<bool>("publish/map_publish_en"', fast_lio_source)
        self.assertIn('nh.param<int>("publish/map_publish_interval"', fast_lio_source)
        self.assertIn("ikdtree.flatten(ikdtree.Root_Node", fast_lio_source)
        self.assertIn("if (map_pub_en && ++map_pub_count >= map_pub_interval)",
                      fast_lio_source)
        self.assertIn("publish_map(pubLaserCloudMap)", fast_lio_source)
        self.assertNotIn("// publish_map(pubLaserCloudMap)", fast_lio_source)

    def test_urdf_provides_imu_and_switchable_truth_tf(self):
        urdf = PACKAGE_ROOT / "urdf" / "turtlebot3_waffle_tare.urdf.xacro"
        text = urdf.read_text()
        self.assertIn('xacro:arg name="publish_odom_tf"', text)
        self.assertIn('libgazebo_ros_imu_sensor.so', text)
        self.assertIn('<topicName>/imu/data</topicName>', text)

    def test_cluttered_world_extends_official_garage(self):
        world = (PACKAGE_ROOT.parent / "autonomous_exploration_development_environment" /
                 "src" / "vehicle_simulator" / "world" / "garage_cluttered.world")
        self.assertTrue(world.is_file())
        root = ET.parse(world).getroot()
        uris = [node.text for node in root.iter("uri")]
        self.assertIn("model://garage", uris)
        models = {node.attrib.get("name") for node in root.iter("model")}
        self.assertIn("garage_overhead_services", models)
        self.assertIn("garage_floor_clutter", models)
        self.assertGreaterEqual(len(list(root.iter("collision"))), 20)
        # The 1,000 clutter pieces are baked into four static meshes.  Keeping
        # one Gazebo model per obstacle type avoids creating 1,000 independent
        # model/link/collision objects while retaining their collision geometry.
        populations = list(root.iter("population"))
        self.assertEqual([], populations)
        aggregate_uris = {
            uri for uri in uris if uri and uri.startswith("model://tare_aggregated_")
        }
        self.assertEqual({
            "model://tare_aggregated_cross_pipes",
            "model://tare_aggregated_junction_boxes",
            "model://tare_aggregated_sprinkler_drops",
            "model://tare_aggregated_hanging_sensors",
        }, aggregate_uris)
        self.assertNotIn("model_count", {
            node.tag for node in root.iter()
        })

        mesh_root = (PACKAGE_ROOT.parent / "autonomous_exploration_development_environment" /
                     "src" / "vehicle_simulator" / "mesh")
        for model_name, mesh_name in (
                ("tare_aggregated_cross_pipes", "cross_pipes.stl"),
                ("tare_aggregated_junction_boxes", "junction_boxes.stl"),
                ("tare_aggregated_sprinkler_drops", "sprinkler_drops.stl"),
                ("tare_aggregated_hanging_sensors", "hanging_sensors.stl")):
            model_dir = mesh_root / model_name
            self.assertTrue((model_dir / "model.sdf").is_file())
            self.assertTrue((model_dir / "model.config").is_file())
            mesh = model_dir / "meshes" / mesh_name
            self.assertTrue(mesh.is_file())
            self.assertGreater(mesh.stat().st_size, 1024)

        # The generator uses 32 facets per cylinder and 12 per box.  These
        # counts assert the intended 300/300/200/200 split, i.e. 1,000 pieces.
        expected_facets = {
            "cross_pipes.stl": 9600,
            "junction_boxes.stl": 3600,
            "sprinkler_drops.stl": 6400,
            "hanging_sensors.stl": 6400,
        }
        for mesh in mesh_root.glob("tare_aggregated_*/meshes/*.stl"):
            with mesh.open() as stream:
                facets = sum(1 for line in stream if line.strip() == "endfacet")
            self.assertEqual(expected_facets[mesh.name], facets)

        launch = PACKAGE_ROOT / "launch" / "tb3_tare_fast_lio_cluttered.launch"
        launch_root = ET.parse(launch).getroot()
        includes = list(launch_root.iter("include"))
        self.assertEqual(1, len(includes))
        forwarded = {arg.attrib.get("name"): arg.attrib.get("value")
                     for arg in includes[0].findall("arg")}
        self.assertEqual("garage_cluttered", forwarded["world_name"])
        self.assertEqual("garage", forwarded["scenario"])
        self.assertEqual("$(arg fast_lio_viz_min_z)",
                         forwarded["fast_lio_viz_min_z"])


if __name__ == "__main__":
    unittest.main()
