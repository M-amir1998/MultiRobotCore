"""Warehouse fleet demo: simulator backend + coordinator + ORCA safety filter.

    ros2 launch multirobotcore_ros warehouse_demo.launch.py real_time_factor:=2.0

The simulator publishes /clock; the coordinator and the safety filter run on
simulation time.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    share = Path(get_package_share_directory("multirobotcore_ros"))
    params = str(share / "config" / "warehouse.yaml")
    rtf = LaunchConfiguration("real_time_factor")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "real_time_factor",
                default_value="1.0",
                description="simulation speed relative to wall-clock time",
            ),
            Node(
                package="multirobotcore_ros",
                executable="fleet_simulator",
                name="fleet_simulator",
                parameters=[params, {"real_time_factor": rtf}],
                output="screen",
            ),
            Node(
                package="multirobotcore_ros",
                executable="warehouse_coordinator",
                name="warehouse_coordinator",
                parameters=[params, {"use_sim_time": True}],
                output="screen",
            ),
            Node(
                package="multirobotcore_ros",
                executable="orca_safety",
                name="orca_safety",
                parameters=[params, {"use_sim_time": True}],
                output="screen",
            ),
        ]
    )
