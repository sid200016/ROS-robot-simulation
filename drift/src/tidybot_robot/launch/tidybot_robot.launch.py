#!/usr/bin/env python3
"""TidyBot simulation launch file.

Launches Gazebo Harmonic with the home world, spawns the robot,
bridges ROS2 <-> Gazebo topics, starts robot_state_publisher,
and runs the autonomous tidying behavior node.

Usage:
  ros2 launch tidybot_robot tidybot_robot.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    pkg_share = get_package_share_directory('tidybot_robot')

    # ── Paths ──────────────────────────────────────────────────────────────
    world_file   = os.path.join(pkg_share, 'worlds', 'home.world')
    xacro_file   = os.path.join(pkg_share, 'urdf',   'tidybot.urdf.xacro')

    # Process xacro → URDF string
    robot_description_content = xacro.process_file(xacro_file).toxml()

    # ── Launch arguments ───────────────────────────────────────────────────
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use Gazebo simulation clock'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    # ── 1. Gazebo Harmonic ─────────────────────────────────────────────────
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', world_file, '-r'],
        output='screen',
        additional_env={'GZ_SIM_RESOURCE_PATH': os.path.join(pkg_share, 'worlds')}
    )

    # ── 2. Robot State Publisher ───────────────────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': use_sim_time,
        }]
    )

    # ── 3. Spawn robot (delayed 5 s to let Gazebo initialise) ──────────────
    spawn_robot = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                name='spawn_tidybot',
                arguments=[
                    '-name',  'tidybot',
                    '-topic', '/robot_description',
                    '-x', '-6.0',
                    '-y',  '0.0',
                    '-z',  '0.05',
                ],
                output='screen',
            )
        ]
    )

    # ── 4. ROS ↔ Gazebo bridge ─────────────────────────────────────────────
    # Bridges: clock, cmd_vel (ROS→GZ), odom (GZ→ROS),
    #          joint_states (GZ→ROS), camera image, lidar scan
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            # Clock
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # Differential drive command
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            # Odometry
            '/model/tidybot/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            # Joint states
            '/world/home_world/model/tidybot/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
            # Camera image
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            # Camera info
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            # LiDAR scan
            '/lidar/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            # IMU
            '/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU',
        ]
    )

    # ── 5. Behavior node (delayed 10 s to let robot spawn & settle) ────────
    behavior_node = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='tidybot_robot',
                executable='tidybot_behavior',
                name='tidybot_behavior',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}],
            )
        ]
    )

    # ── 6. Static TF: odom → base_footprint (until odometry plugin publishes) ─
    # The diff-drive plugin in Gazebo publishes odom→base_footprint TF,
    # but we add a static one as fallback so RViz doesn't complain at startup.
    static_tf_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_odom_base',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_footprint'],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        use_sim_time_arg,
        gazebo,
        robot_state_publisher,
        static_tf_odom,
        spawn_robot,
        bridge,
        behavior_node,
    ])
