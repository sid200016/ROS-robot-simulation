from pathlib import Path

from ament_index_python.packages import get_package_share_directory # pyright: ignore[reportMissingImports]
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, TimerAction # pyright: ignore[reportMissingImports]
from launch.event_handlers import OnProcessExit # pyright: ignore[reportMissingImports]
from launch.substitutions import Command, FindExecutable, LaunchConfiguration # pyright: ignore[reportMissingImports]
from launch_ros.actions import Node # pyright: ignore[reportMissingImports]
from launch_ros.parameter_descriptions import ParameterValue # pyright: ignore[reportMissingImports]


def generate_launch_description():
    package_share = Path(get_package_share_directory("ros_sim_robot"))
    default_urdf = package_share / "urdf" / "robot_fin.urdf"

    urdf_path = LaunchConfiguration("urdf_path")
    entity_name = LaunchConfiguration("entity_name")
    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    z = LaunchConfiguration("z")

    gazebo = ExecuteProcess(
        cmd=[
            "ros2",
            "launch",
            "ros_gz_sim",
            "gz_sim.launch.py",
            "gz_args:=-r empty.sdf",
        ],
        output="screen",
    )

    spawn = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "launch",
                    "ros_gz_sim",
                    "gz_spawn_model.launch.py",
                    "world:=empty",
                    ["file:=", urdf_path],
                    ["entity_name:=", entity_name],
                    ["x:=", x],
                    ["y:=", y],
                    ["z:=", z],
                ],
                output="screen",
            )
        ],
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": ParameterValue(
                    Command([FindExecutable(name="xacro"), " ", urdf_path]),
                    value_type=str,
                ),
                "use_sim_time": True,
            }
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "diff_drive_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    start_diff_drive_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[diff_drive_controller_spawner],
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "urdf_path",
                default_value=str(default_urdf),
                description="Absolute path to the URDF file to spawn.",
            ),
            DeclareLaunchArgument(
                "entity_name",
                default_value="robot",
                description="Entity name inside Gazebo.",
            ),
            DeclareLaunchArgument("x", default_value="0.0"),
            DeclareLaunchArgument("y", default_value="0.0"),
            DeclareLaunchArgument("z", default_value="0.1"),
            clock_bridge,
            robot_state_publisher,
            gazebo,
            spawn,
            joint_state_broadcaster_spawner,
            start_diff_drive_controller,
        ]
    )
