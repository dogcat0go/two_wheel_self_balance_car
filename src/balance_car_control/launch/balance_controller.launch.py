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
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _resolve_config_path(context):
    mode = LaunchConfiguration("control_mode").perform(context)
    pkg_dir = get_package_share_directory("balance_car_control")
    if mode == "lqr":
        return os.path.join(pkg_dir, "config", "balance_lqr.yaml")
    return os.path.join(pkg_dir, "config", "balance_controller.yaml")


def launch_setup(context, *args, **kwargs):
    config_override = LaunchConfiguration("config_file").perform(context)
    if config_override:
        config_path = config_override
    else:
        config_path = _resolve_config_path(context)

    use_sim_time = LaunchConfiguration("use_sim_time")
    enabled = LaunchConfiguration("enabled")
    control_mode = LaunchConfiguration("control_mode")

    balance_controller = Node(
        package="balance_car_control",
        executable="balance_controller_node",
        name="balance_controller_node",
        output="screen",
        parameters=[
            config_path,
            {
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "enabled": ParameterValue(enabled, value_type=bool),
                "control_mode": ParameterValue(control_mode, value_type=str),
            },
        ],
    )

    slope_dyn_obs = Node(
        package="balance_car_control",
        executable="slope_dyn_obs_node",
        name="slope_dyn_obs",
        output="screen",
        parameters=[{
            "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
        }],
    )

    return [
        TimerAction(
            period=3.0,
            actions=[balance_controller, slope_dyn_obs],
        )
    ]


def generate_launch_description():
    pkg_dir = get_package_share_directory("balance_car_control")
    default_config_file = os.path.join(
        pkg_dir,
        "config",
        "balance_controller.yaml",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "control_mode",
            default_value="pid",
            description="Balance control backend: 'pid' (cascade) or 'lqr' (full-state).",
            choices=["pid", "lqr"],
        ),
        DeclareLaunchArgument(
            "config_file",
            default_value="",
            description="Override config YAML path. Empty = auto-select from control_mode.",
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
        OpaqueFunction(function=launch_setup),
    ])
