'''
Author: LCOIT dogcat.let@gmail.com
Date: 2026-05-27 23:57:35
LastEditors: LCOIT dogcat.let@gmail.com
LastEditTime: 2026-05-29 00:44:14
FilePath: /two_wheel_self_balance_car/src/balance_car_control/launch/balance_controller.launch.py
Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
'''
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_dir = get_package_share_directory("balance_car_control")
    default_config_file = os.path.join(
        pkg_dir,
        "config",
        "balance_controller.yaml",
    )

    config_file = LaunchConfiguration("config_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    enabled = LaunchConfiguration("enabled")

    balance_controller = Node(
        package="balance_car_control",
        executable="balance_controller_node",
        name="balance_controller_node",
        output="screen",
        parameters=[
            config_file,
            {
                # Keep this explicit so the node follows Gazebo /clock even if
                # the YAML is copied or edited later.
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                # Default false: start in observe-only mode, then enable after
                # IMU sign and wheel direction are confirmed.
                "enabled": ParameterValue(enabled, value_type=bool),
            },
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value=default_config_file,
            description="Path to balance controller parameter YAML.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use Gazebo simulation time from /clock.",
        ),
        DeclareLaunchArgument(
            "enabled",
            default_value="true",
            description="Whether to publish non-zero wheel commands.",
        ),
        TimerAction(
            # Give Gazebo, /clock, ros2_control controllers, and the IMU topic
            # time to start before the balance loop begins subscribing/publishing.
            period=3.0,
            actions=[balance_controller],
        ),
    ])
