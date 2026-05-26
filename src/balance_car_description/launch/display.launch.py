import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    pkg_dir = get_package_share_directory("balance_car_description")

    urdf_xacro = os.path.join(pkg_dir, "urdf", "two_wheel_balance_65mm.urdf.xacro")
    rviz_config = os.path.join(pkg_dir, "rviz", "display.rviz")

    robot_desc = xacro.process_file(urdf_xacro).toxml()

    joint_state_pub = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        parameters=[{
            "robot_description": robot_desc,
            "use_sim_time": False,
        }],
    )

    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": robot_desc,
            "use_sim_time": False,
        }],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": False}],
    )

    return LaunchDescription([
        joint_state_pub,
        robot_state_pub,
        rviz,
    ])
