from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration


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
            "gz_args:=empty.sdf",
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
            gazebo,
            spawn,
        ]
    )
