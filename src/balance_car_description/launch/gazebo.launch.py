import os
import re
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
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
    # world_file = os.path.join(pkg_dir, "worlds", "imu_debug.world")
    world_file = os.path.join(pkg_dir, "worlds", "empty.world")
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    use_rviz = LaunchConfiguration("use_rviz", default="true")
    world = LaunchConfiguration("world", default=world_file)
    pause = LaunchConfiguration("pause", default="false")
    spawn_x = LaunchConfiguration("spawn_x", default="0.0")
    spawn_y = LaunchConfiguration("spawn_y", default="0.0")
    spawn_z = LaunchConfiguration("spawn_z", default="0.034")
    spawn_yaw = LaunchConfiguration("spawn_yaw", default="0.0")
    # 跑 headless（关掉 gzclient）能省 30-50% CPU，调参时强烈推荐。
    # 需要看车体姿态时再 gui:=true。
    gui = LaunchConfiguration("gui", default="true")
    # "effort" or "velocity". URDF 与 ros2_control yaml 同时暴露两套接口，
    # 这里决定 spawner 启动哪一个（另一条仍可手动 ros2 control load_controller 拉起）。
    controller_type = LaunchConfiguration("controller_type", default="effort")

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
            "gui": gui,
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
            "-x", spawn_x,
            "-y", spawn_y,
            "-z", spawn_z,
            "-Y", spawn_yaw,
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

    load_wheel_velocity_ctrl = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["wheel_velocity_controller", "--switch-timeout", "20.0"],
        output="screen",
        condition=IfCondition(
            PythonExpression(["'", controller_type, "' == 'velocity'"])
        ),
    )

    load_wheel_effort_ctrl = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["wheel_effort_controller", "--switch-timeout", "20.0"],
        output="screen",
        condition=IfCondition(
            PythonExpression(["'", controller_type, "' == 'effort'"])
        ),
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
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Whether to launch gzclient (Gazebo GUI). Set false to save CPU when tuning.",
        ),
        DeclareLaunchArgument(
            "controller_type",
            default_value="effort",
            description="Which wheel controller to spawn: 'effort' or 'velocity'.",
        ),
        DeclareLaunchArgument(
            "spawn_x",
            default_value="0.0",
            description="Spawn X [m]. gentle_slope / balance_pid_test 上坡前: -4.5；岭台: -0.5。",
        ),
        DeclareLaunchArgument("spawn_y", default_value="0.0"),
        DeclareLaunchArgument(
            "spawn_z",
            default_value="0.034",
            description="Spawn Z [m] 轮心。上坡前 0.034；岭台 gentle 0.209 / pid 0.315。",
        ),
        DeclareLaunchArgument("spawn_yaw", default_value="0.0"),
        gazebo,
        robot_state_pub,
        spawn_entity,
        load_joint_state_bc,
        load_wheel_velocity_ctrl,
        load_wheel_effort_ctrl,
        load_imu_bc,
        # rviz,
    ])
