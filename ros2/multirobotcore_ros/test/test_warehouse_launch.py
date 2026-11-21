"""End-to-end launch test: simulator + coordinator + ORCA filter deliver tasks
without contacts.

Runs a small fleet faster than real time and asserts, from the nodes' JSON
status topics, that tasks are delivered and that the simulator recorded no
robot-robot or robot-shelf contact.
"""

import json
import time
import unittest
from pathlib import Path

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import String

FLEET = {
    "num_robots": 6,
    "layout.shelf_block_columns": 3,
    "layout.shelf_block_rows": 3,
    "seed": 3,
}


@pytest.mark.launch_test
def generate_test_description():
    params = str(Path(get_package_share_directory("multirobotcore_ros")) / "config" / "warehouse.yaml")
    sim = launch_ros.actions.Node(package="multirobotcore_ros", executable="fleet_simulator",
                                  name="fleet_simulator",
                                  parameters=[params, FLEET, {"real_time_factor": 4.0}])
    coordinator = launch_ros.actions.Node(package="multirobotcore_ros", executable="warehouse_coordinator",
                                          name="warehouse_coordinator",
                                          parameters=[params, FLEET, {"use_sim_time": True, "initial_tasks": 6,
                                                                       "task_rate": 0.0}])
    safety = launch_ros.actions.Node(package="multirobotcore_ros", executable="orca_safety", name="orca_safety",
                                     parameters=[params, FLEET, {"use_sim_time": True}])
    return launch.LaunchDescription([sim, coordinator, safety, launch_testing.actions.ReadyToTest()])


class TestWarehouseFleet(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_tasks_are_delivered_without_contacts(self):
        node = rclpy.create_node("warehouse_test_observer")
        latest = {}
        node.create_subscription(String, "/fleet/status", lambda m: latest.update(fleet=json.loads(m.data)), 10)
        node.create_subscription(String, "/fleet/sim_status", lambda m: latest.update(sim=json.loads(m.data)), 10)
        deadline = time.monotonic() + 150.0
        try:
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.2)
                fleet = latest.get("fleet", {})
                if fleet.get("tasks_completed", 0) >= 6:
                    break
        finally:
            node.destroy_node()
        self.assertIn("fleet", latest, "no status from the coordinator")
        self.assertIn("sim", latest, "no status from the simulator")
        self.assertGreaterEqual(latest["fleet"]["tasks_completed"], 6, latest)
        self.assertEqual(latest["fleet"]["planning_failures"], 0, latest)
        self.assertEqual(latest["sim"]["robot_contacts"], 0, latest)
        self.assertEqual(latest["sim"]["obstacle_contacts"], 0, latest)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -2, -15])
