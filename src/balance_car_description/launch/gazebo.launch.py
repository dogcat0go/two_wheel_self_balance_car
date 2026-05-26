import os
import re
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def load_robot_description(xacro_path):
    robot_xml = xacro.process_file(xacro_path).toxml()
    robot_xml = re.sub(r"^\s*<\?xml[^>]*\?>\s*", "", robot_xml)
    robot_xml = re.sub(r"<!--.*?-->", "", robot_xml, flags=re.DOTALL)
    return robot_xml.strip()


def write_spawn_urdf(robot_xml):
    path = os.path.join(tempfile.gettempdir(), "two_wheel_balance_65mm_spawn.urdf")
    with open(path, "w", encoding="utf-8") as urdf_file:
        urdf_file.write(robot_xml)
    return path


def generate_launch_description():
    pkg_dir = get_package_share_directory("balance_car_description")

    # Paths
    urdf_xacro = os.path.join(pkg_dir, "urdf", "two_wheel_balance_65mm.urdf.xacro")
    world_file = os.path.join(pkg_dir, "worlds", "empty.world")

    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    use_rviz = LaunchConfiguration("use_rviz", default="true")
    world = LaunchConfiguration("world", default=world_file)
    pause = LaunchConfiguration("pause", default="false")

    # Generate a clean robot_description once. Gazebo Humble can fail if the
    # full XML declaration/comment block is passed through ROS parameter args.
    robot_desc = load_robot_description(urdf_xacro)
    spawn_urdf = write_spawn_urdf(robot_desc)

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory("gazebo_ros"),
                "launch", "gazebo.launch.py"
            )
        ]),
        launch_arguments={
            "world": world,
            "pause": pause,
        }.items(),
    )

    # Robot State Publisher
    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": robot_desc,
            "use_sim_time": use_sim_time,
        }],
    )

    # Spawn robot in Gazebo
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-file", spawn_urdf,
            "-entity", "two_wheel_balance_65mm",
            "-x", "0.0", "-y", "0.0", "-z", "0.034",
        ],
        output="screen",
    )

    # Load controllers
    load_joint_state_bc = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--switch-timeout", "20.0"],
        output="screen",
    )

    load_wheel_ctrl = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["wheel_velocity_controller", "--switch-timeout", "20.0"],
        output="screen",
    )

    load_imu_bc = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["imu_sensor_broadcaster", "--switch-timeout", "20.0"],
        output="screen",
    )

    # RViz
    # rviz = Node(
    #     package="rviz2",
    #     executable="rviz2",
    #     arguments=["-d", rviz_config],
    #     condition=IfCondition(use_rviz),
    #     parameters=[{"use_sim_time": use_sim_time}],
    # )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("world", default_value=world_file),
        DeclareLaunchArgument("pause", default_value="false"),
        gazebo,
        robot_state_pub,
        spawn_entity,
        load_joint_state_bc,
        load_wheel_ctrl,
        load_imu_bc,
        # rviz,
    ])
