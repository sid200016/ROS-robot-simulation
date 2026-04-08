from pathlib import Path

from ament_index_python.packages import get_package_share_directory # pyright: ignore[reportMissingImports]
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, TimerAction # pyright: ignore[reportMissingImports]
from launch.event_handlers import OnProcessExit # pyright: ignore[reportMissingImports]
from launch.substitutions import Command, FindExecutable, LaunchConfiguration # pyright: ignore[reportMissingImports]
from launch_ros.actions import Node # pyright: ignore[reportMissingImports]
from launch_ros.parameter_descriptions import ParameterValue # pyright: ignore[reportMissingImports]
from ros_gz_bridge.actions import RosGzBridge # pyright: ignore[reportMissingImports]
from launch.actions import IncludeLaunchDescription # pyright: ignore[reportMissingImports]
from launch.launch_description_sources import PythonLaunchDescriptionSource # pyright: ignore[reportMissingImports]

def generate_launch_description():
    package_share = Path(get_package_share_directory("ros_sim_robot"))
    default_urdf = package_share / "urdf" / "robot_fin.urdf"
    default_world = package_share / "worlds" / "sensor_empty.sdf"
    world_name = "sensor_empty"

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
            f"gz_args:=-r {default_world}",
        ],
        output="screen",
    )

    spawn = TimerAction(
        period=0.1,
        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "launch",
                    "ros_gz_sim",
                    "gz_spawn_model.launch.py",
                    f"world:={world_name}",
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

    clock_and_sensor_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        "/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",
        "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
        "/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
        "/imu/data_raw@sensor_msgs/msg/Imu[gz.msgs.IMU"],
        output="screen",
    )

    set_pose_bridge = RosGzBridge(
        bridge_name="set_pose_bridge",
        config_file="",
        container_name="ros_gz_container",
        create_own_container=False,
        namespace="",
        use_composition=False,
        use_respawn=False,
        log_level="warn",
        extra_bridge_params={
            "bridges": {
                "bridge_0": {
                    "service_name": f"/world/{world_name}/set_pose",
                    "ros_type_name": "ros_gz_interfaces/srv/SetEntityPose",
                    "gz_req_type_name": "gz.msgs.Pose",
                    "gz_rep_type_name": "gz.msgs.Boolean",
                },
            },
            "bridge_names": ["bridge_0"],
        },
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
    ekf_filter = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[package_share / "config" / "ekf.yaml"],
    )


    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(Path(get_package_share_directory("slam_toolbox")) / "launch" / "online_async_launch.py")
        ),
        launch_arguments={
            "slam_params_file": str(package_share / "config" / "slam_toolbox.yaml"),
        }.items(),
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

    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "left_arm_controller",
            "right_arm_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    left_arm_init_pose = ExecuteProcess(
    cmd=[
        "ros2", "topic", "pub", "--once",
        "/left_arm_controller/joint_trajectory",
        "trajectory_msgs/msg/JointTrajectory",
        "{joint_names: ['left_arm_shoulder_joint', 'left_arm_elbow_joint', 'left_arm_wrist_joint'], "
        "points: [{positions: [-0.6, -0.7, -0.5], time_from_start: {sec: 2}}]}"
    ],
    output="screen",)

    right_arm_init_pose = ExecuteProcess(
    cmd=[
        "ros2", "topic", "pub", "--once",
        "/right_arm_controller/joint_trajectory",
        "trajectory_msgs/msg/JointTrajectory",
        "{joint_names: ['right_arm_shoulder_joint', 'right_arm_elbow_joint', 'right_arm_wrist_joint'], "
        "points: [{positions: [0.6, 0.7, 0.5], time_from_start: {sec: 2}}]}"
    ],
    output="screen",)

    
    waypoint_robot = Node(
        package="ros_sim_robot",
        executable="waypoint_node.py",
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    start_diff_drive_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[diff_drive_controller_spawner],
        )
    )
    start_joint_trajectory_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=diff_drive_controller_spawner,
            on_exit=[joint_trajectory_controller_spawner],
        )
    )
    start_ekf = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_trajectory_controller_spawner,
            on_exit=[ekf_filter],
        )
    )

    start_SLAM = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_trajectory_controller_spawner,
            on_exit=[slam_launch],
        )
    )

    move_to_init_pose = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_trajectory_controller_spawner,
            on_exit=[left_arm_init_pose, right_arm_init_pose],
        )
    )

    start_waypoint_robot = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_trajectory_controller_spawner,
            on_exit=[TimerAction(period=3.0, actions=[waypoint_robot])],
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
            DeclareLaunchArgument("x", default_value="-3.0"),
            DeclareLaunchArgument("y", default_value="6.0"),
            DeclareLaunchArgument("z", default_value="0.2"),
            clock_and_sensor_bridge,
            set_pose_bridge,
            robot_state_publisher,
            gazebo,
            spawn,
            joint_state_broadcaster_spawner,
            start_diff_drive_controller,
            start_joint_trajectory_controller,
            start_ekf, 
            start_SLAM,
            move_to_init_pose,
            start_waypoint_robot
        ]
    )
