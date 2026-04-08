#!/usr/bin/env python3

import csv
import math
from datetime import datetime
from pathlib import Path as pt

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry, Path
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

WORLD_START = (-3.0, 6.0, 0.0)
TARGET_CUBE = ["obs6"]
BOX_DROP_WORLD = (-1.5, 7.5, 0.08, 0.0)

KNOWN_CUBES = {
    "obs2": (3.0, 2.5),
    "obs3": (3.0, -2.5),
    "obs4": (-3.0, 1.5),
    "obs5": (-3.0, -1.5),
    "obs6": (3.0, 6.0),
}

RIGHT_ARM_HOME = [0.6, 0.7, 0.5]
RIGHT_ARM_PREGRASP = [0.3, -1.2, 0.9]
RIGHT_ARM_PICK = [0.5, -1.6, 1.1]
RIGHT_ARM_CARRY = [-0.3, 0.9, -0.4]

CUBE_MISSIONS = {
    "obs6": {
        "pick_route_world": [
            (0.0, 6.0, 0.0),
            (2.0, 6.0, 0.0),
            (2.12, 6.2, math.pi / 2.0),
        ],
        "drop_route_world": [
            (2.0, 5.55, math.pi),
            (0.0, 5.55, math.pi),
            (-1.5, 6.80, math.pi),
        ],
        "drop_pose_world": BOX_DROP_WORLD,
    },
}

ARM_MOVE_TIME = 2.5
CUBE_HALF_HEIGHT = 0.16
POSE_REQUEST_TIMEOUT = 1.0

RIGHT_ARM_MOUNT_Y = -(0.42 / 2.0 + 0.025)
RIGHT_ARM_MOUNT_Z = 0.10 + 0.10 + (0.10 / 2.0 - 0.025)

WPs = [
    [0.0, 6.0, -math.pi / 4.0],
    [0.0, 6.0, -math.pi / 2.0],
    [0.0, 2.5, -math.pi / 2.0],
    [3.0, 2.5, 0.0],
    [3.0, 2.5, math.pi],
    [0.0, 2.5, math.pi],
    [0.0, 2.5, -math.pi/2.0], 
    [0.0, 1.5, -math.pi/2.0],
    [-3.0, 1.5, math.pi],
    [-3.0, 1.5, 0.0],
    [0.0, 1.5, 0.0],
    [0.0, 1.5, -math.pi/2.0],
    [0.0, -1.5, -math.pi/2.0],
    [-3.0, -1.5, math.pi],
    [-3.0, -1.5, 0.0],
    [0.0, -1.5, 0.0],
    [0.0, -1.5, -math.pi/2.0],
    [0.0, -2.5, -math.pi/2.0],
    [3.0, -2.5, 0.0],
    [3.0, -2.5, math.pi],
    [0.0, -2.5, math.pi], 
    [0.0, -2.5, -math.pi/2.0],
    [0.0, -6.0, -math.pi/2.0],
    [3.0, -6.0, 0.0],
    [3.0, -6.0, math.pi],
    [0.0, -6.0, math.pi],
    [-3.0, -6.0, math.pi],
    [-3.0, -6.0, 0.0],
    [0.0, -6.0, 0.0],
    [0.0, -6.0, math.pi/2.0],  
    [0.0, 6.0, math.pi/2.0],
    [-3.0, 6.0, math.pi],
    [3.0, 6.0, 0.0],
]
 
 
class WaypointNode(Node):
    def __init__(self):
        super().__init__("waypoint_node")
        self.WPs_world = [tuple(waypoint) for waypoint in WPs]
        self.WPs = []
        self.scan_msg = None
        self.odom_msg = None
        self.map_pose = None
        self.map_start = None
        self.mode = "FORWARD"
        self.turn_dir = 1.0
        self.front_min = float("inf")
        self.front_left_min = float("inf")
        self.front_right_min = float("inf")
        self.left_side_min = float("inf")
        self.right_side_min = float("inf")
        self.back_min = float("inf")

        self.self_side_ignore = 0.38


        self.distance_traveled = 0.0
        self.prev_xy = None
        self.rooms_visited = set()
        self.current_room = "NORTH"
        self.localization_frame = "map"
        self.base_frame = "base_link"
        self.last_tf_error_ns = 0

        self.position_tolerance = 0.06
        self.yaw_tolerance = 0.05
        self.turn_only_threshold = 0.22
        self.max_linear = 0.5
        self.max_angular = 0.25
        self.nav_front_slow = 0.95
        self.nav_front_stop = 0.62
        self.nav_side_slow = 0.82
        self.nav_side_stop = 0.64

        self.reverse_distance = 0.45
        self.reverse_linear_limit = 0.22
        self.reverse_back_stop = 0.65
        self.reverse_back_slow = 1.20
        self.cube_idx = 0
        self.target_cube = TARGET_CUBE[self.cube_idx]
        if self.target_cube not in CUBE_MISSIONS:
            known = ", ".join(sorted(CUBE_MISSIONS))
            raise ValueError(
                f"Unsupported TARGET_CUBE {self.target_cube!r}. Choose one of: {known}"
            )
        self.mission = CUBE_MISSIONS[self.target_cube]
        self.right_arm_target = list(RIGHT_ARM_HOME)
        self.right_arm_goal_future = None
        self.cube_attached = False
        self.cube_drop_requested = False
        self.set_pose_client = None
        self.set_pose_service_name = "/world/sensor_empty/set_pose"
        self.set_pose_future = None
        self.set_pose_request_started_ns = 0
        self.last_cube_update_ns = 0
        self.last_pose_error_ns = 0

        self.mission_state = "NAV_TO_CUBE"
        self.state_started_ns = 0
        self.state_entry_done = False
        self.backout_start_xy = None
        self.backout_hold_yaw = 0.0

        self.pick_route_map = []
        self.box_route_map = []
        self.active_route = []
        self.active_route_name = "IDLE"
        self.route_index = 0
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.scan_sub = self.create_subscription(LaserScan, "/scan", self.scan_cb, 10)
        self.odom_sub = self.create_subscription(
            Odometry, "/odometry/filtered", self.odom_cb, 30
        )

        self.cmd_pub = self.create_publisher(
            TwistStamped, "/diff_drive_controller/cmd_vel", 10
        )
        self.right_arm_traj_pub = self.create_publisher(
            JointTrajectory, "/right_arm_controller/joint_trajectory", 10
        )
        self.right_arm_action = ActionClient(
            self,
            FollowJointTrajectory,
            "/right_arm_controller/follow_joint_trajectory",
        )
        self.path_pub = self.create_publisher(Path, "/path", 10)
        self.set_pose_client = self.create_client(
            SetEntityPose,
            self.set_pose_service_name,
        )

        self.path_msg = Path()
        self.path_msg.header.frame_id = self.localization_frame

        self.control_timer = self.create_timer(0.02, self.control_loop)
        self.status_timer = self.create_timer(1.0, self.status_loop)

        workspace_root = pt(__file__).resolve().parents[4]
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.run_dir = workspace_root / "output" / stamp
        self.run_dir.mkdir(parents=True, exist_ok=True)

        self.odom_file = (self.run_dir / "odom.csv").open("w", newline="")
        self.odom_writer = csv.writer(self.odom_file)
        self.odom_writer.writerow(
            ["t", "x", "y", "yaw", "state", "front", "back", "left", "right"]
        )

        self.png_timer = self.create_timer(2.0, self.save_path_png)
        self.odom_flush_counter = 0

    def clamp(self, value, lo, hi):
        return max(lo, min(hi, value))

    def wrap_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def set_active_route(self, route, route_name):
        self.active_route = list(route)
        self.route_index = 0
        self.active_route_name = route_name
        self.mode = route_name

    def waypoint_triplet(self, waypoint, index):
        if len(waypoint) != 3:
            raise ValueError(
                f"Waypoint {index} must contain [x, y, yaw], got {waypoint!r}"
            )
        return tuple(waypoint)

    def current_pose(self):
        return self.map_pose

    def current_world_pose(self):
        return self.map_pose_to_world(*self.current_pose())

    def refresh_map_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.localization_frame,
                self.base_frame,
                Time(),
            )
        except TransformException as exc:
            now_ns = self.get_clock().now().nanoseconds
            if now_ns - self.last_tf_error_ns > 2_000_000_000:
                self.get_logger().warn(
                    f"Waiting for {self.localization_frame}->{self.base_frame} TF: {exc}"
                )
                self.last_tf_error_ns = now_ns
            return False

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        self.map_pose = (
            translation.x,
            translation.y,
            self.yaw_from_quat(rotation),
        )
        return True

    def pose_errors(self, gx, gy, gyaw):
        x, y, yaw = self.current_pose()
        dx = gx - x
        dy = gy - y
        distance = math.hypot(dx, dy)
        heading_error = self.wrap_angle(math.atan2(dy, dx) - yaw)
        final_yaw_error = self.wrap_angle(gyaw - yaw)
        return distance, heading_error, final_yaw_error

    def start_explore_route(self):
        if not self.WPs:
            self.get_logger().warn("Explore waypoint list is empty; staying stopped")
            return False

        self.set_active_route(self.WPs, "EXPLORE_ROUTE")
        self.get_logger().info(
            f"Starting explore route with {len(self.WPs)} map-frame waypoints"
        )
        return True

    def transition_to(self, new_state):
        self.mission_state = new_state
        self.state_started_ns = self.get_clock().now().nanoseconds
        self.state_entry_done = False
        self.backout_start_xy = None
        self.get_logger().info(f"Transition -> {new_state}")

    def entered_state(self):
        if self.state_entry_done:
            return False
        self.state_entry_done = True
        return True

    def state_elapsed(self):
        return (self.get_clock().now().nanoseconds - self.state_started_ns) / 1e9

    def world_to_map(self, wx, wy):
        wsx, wsy, wsyaw = WORLD_START
        msx, msy, msyaw = self.map_start
        dx_w = wx - wsx
        dy_w = wy - wsy
        dtheta = msyaw - wsyaw
        mx = msx + math.cos(dtheta) * dx_w - math.sin(dtheta) * dy_w
        my = msy + math.sin(dtheta) * dx_w + math.cos(dtheta) * dy_w
        return mx, my

    def map_to_world(self, mx, my):
        wsx, wsy, wsyaw = WORLD_START
        msx, msy, msyaw = self.map_start
        dtheta = msyaw - wsyaw
        dx_m = mx - msx
        dy_m = my - msy
        dx_w = math.cos(dtheta) * dx_m + math.sin(dtheta) * dy_m
        dy_w = -math.sin(dtheta) * dx_m + math.cos(dtheta) * dy_m
        return wsx + dx_w, wsy + dy_w

    def world_pose_to_map(self, wx, wy, wyaw):
        mx, my = self.world_to_map(wx, wy)
        dtheta = self.map_start[2] - WORLD_START[2]
        myaw = self.wrap_angle(wyaw + dtheta)
        return (mx, my, myaw)

    def map_pose_to_world(self, mx, my, myaw):
        wx, wy = self.map_to_world(mx, my)
        dtheta = self.map_start[2] - WORLD_START[2]
        wyaw = self.wrap_angle(myaw - dtheta)
        return (wx, wy, wyaw)

    def build_map_routes(self):
        self.pick_route_map = [
            self.world_pose_to_map(wx, wy, wyaw)
            for wx, wy, wyaw in self.mission["pick_route_world"]
        ]
        self.box_route_map = [
            self.world_pose_to_map(wx, wy, wyaw)
            for wx, wy, wyaw in self.mission["drop_route_world"]
        ]
        self.WPs = [
            self.world_pose_to_map(*self.waypoint_triplet(waypoint, index + 1))
            for index, waypoint in enumerate(self.WPs_world)
        ]
        self.set_active_route(self.pick_route_map, "NAV_TO_CUBE_ROUTE")

    def scan_cb(self, msg):
        self.scan_msg = msg
        self.front_min = self.sector_min(msg, -25.0, 25.0)
        self.front_left_min = self.sector_min(msg, 25.0, 70.0)
        self.front_right_min = self.sector_min(msg, -70.0, -25.0)
        self.left_side_min = self.sector_min(msg, 70.0, 150.0, self.self_side_ignore)
        self.right_side_min = self.sector_min(msg, -150.0, -70.0, self.self_side_ignore)
        self.back_min = self.sector_min(msg, 150.0, -150.0)

    def yaw_from_quat(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def quaternion_from_yaw(self, yaw):
        half = yaw / 2.0
        return (0.0, 0.0, math.sin(half), math.cos(half))

    def save_path_png(self):
        if not self.path_msg.poses:
            return

        xs, ys, yaws = self.current_pose()

        fig, ax = plt.subplots(figsize=(6, 10))
        ax.plot(xs, ys, linewidth=2)

        ax.set_xlabel("x (m)")
        ax.set_ylabel("y (m)")
        ax.set_title("Robot Path in robot initial frame")
        ax.grid(True)
        ax.axis("equal")
        fig.tight_layout()
        plt.legend()
        fig.savefig(self.run_dir / "path_live.png", dpi=180)
        plt.close(fig)

        fig, ax = plt.subplots(figsize=(6, 10))
        wx, wy, wyaw = self.current_world_pose()
        ax.plot(wx, wy, linewidth=2)
        
        ax.set_xlabel("x (m)")
        ax.set_ylabel("y (m)")
        ax.set_title("Robot Path in world frame")
        ax.grid(True)
        ax.axis("equal")
        fig.tight_layout()
        plt.legend()
        fig.savefig(self.run_dir / "path_live_world.png", dpi=180)
        plt.close(fig)


    def odom_cb(self, msg):
        self.odom_msg = msg

        if not self.refresh_map_pose():
            return

        x, y, yaw = self.current_pose()

        if self.map_start is None:
            self.map_start = self.current_pose()
            self.build_map_routes()
            self.transition_to("NAV_TO_CUBE")
            self.get_logger().info(f"map_start set to {self.map_start}")

        _, world_y, _ = self.current_world_pose()

        if self.prev_xy is not None:
            self.distance_traveled += math.hypot(x - self.prev_xy[0], y - self.prev_xy[1])
        self.prev_xy = (x, y)

        self.current_room = self.classify_room(world_y)
        if self.current_room == "north":
            self.rooms_visited.add("north")
        elif self.current_room == "south":
            self.rooms_visited.add("south")

        pose = PoseStamped()
        pose.header.stamp = msg.header.stamp
        pose.header.frame_id = self.localization_frame
        pose.pose.position.x = x
        pose.pose.position.y = y
        qx, qy, qz, qw = self.quaternion_from_yaw(yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        self.path_msg.header.stamp = msg.header.stamp
        self.path_msg.poses.append(pose)
        self.path_pub.publish(self.path_msg)

        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        self.odom_writer.writerow(
            [
                stamp,
                x,
                y,
                yaw,
                self.mission_state,
                self.front_min,
                self.back_min,
                self.left_side_min,
                self.right_side_min,
            ]
        )

        self.odom_flush_counter += 1
        if self.odom_flush_counter >= 10:
            self.odom_file.flush()
            self.odom_flush_counter = 0

    def control_loop(self):
        self.pump_set_pose_transport()

        if self.odom_msg is None or self.scan_msg is None:
            self.stop_robot()
            return

        if not self.refresh_map_pose() or self.map_start is None:
            self.stop_robot()
            return

        if self.cube_attached:
            self.update_attached_cube_pose()

        if self.mission_state == "NAV_TO_CUBE":
            if self.follow_active_route():
                self.transition_to("ARM_PREGRASP")
            return

        if self.mission_state == "ARM_PREGRASP":
            self.stop_robot()
            if self.entered_state():
                self.publish_right_arm_pose(RIGHT_ARM_PREGRASP, use_action=True)
            elif self.should_refresh_arm_command():
                self.publish_right_arm_pose(RIGHT_ARM_PREGRASP)
            if self.state_elapsed() >= ARM_MOVE_TIME:
                self.transition_to("ARM_PICK")
            return

        if self.mission_state == "ARM_PICK":
            self.stop_robot()
            if self.entered_state():
                self.publish_right_arm_pose(RIGHT_ARM_PICK, use_action=True)
            elif self.should_refresh_arm_command():
                self.publish_right_arm_pose(RIGHT_ARM_PICK)
            if self.state_elapsed() >= ARM_MOVE_TIME:
                self.transition_to("ATTACH_CUBE")
            return

        if self.mission_state == "ATTACH_CUBE":
            self.stop_robot()
            if self.entered_state():
                self.cube_attached = True
            self.update_attached_cube_pose(force=True)
            if self.state_elapsed() >= 0.6 and not self.pose_transport_busy():
                self.transition_to("ARM_CARRY")
            elif self.state_elapsed() >= 2.0:
                self.get_logger().warn("Attach timed out; continuing mission")
                self.clear_pose_transport()
                self.transition_to("ARM_CARRY")
            return

        if self.mission_state == "ARM_CARRY":
            self.stop_robot()
            if self.entered_state():
                self.publish_right_arm_pose(RIGHT_ARM_CARRY, use_action=True)
            elif self.should_refresh_arm_command():
                self.publish_right_arm_pose(RIGHT_ARM_CARRY)
            if self.state_elapsed() >= ARM_MOVE_TIME:
                self.transition_to("BACK_OUT")
            return

        if self.mission_state == "BACK_OUT":
            if self.execute_backout():
                self.set_active_route(self.box_route_map, "NAV_TO_BOX_ROUTE")
                self.transition_to("NAV_TO_BOX")
            return

        if self.mission_state == "NAV_TO_BOX":
            if self.follow_active_route():
                self.transition_to("ARM_DROP")
            return

        if self.mission_state == "ARM_DROP":
            self.stop_robot()
            if self.entered_state():
                self.publish_right_arm_pose(RIGHT_ARM_PICK, use_action=True)
            elif self.should_refresh_arm_command():
                self.publish_right_arm_pose(RIGHT_ARM_PICK)
            if self.state_elapsed() >= ARM_MOVE_TIME:
                self.transition_to("DETACH_CUBE")
            return

        if self.mission_state == "DETACH_CUBE":
            self.stop_robot()
            drop_pose_world = self.mission.get("drop_pose_world", BOX_DROP_WORLD)
            if self.entered_state():
                self.cube_attached = False
                self.cube_drop_requested = self.request_cube_pose(*drop_pose_world)
            if not self.cube_drop_requested:
                self.cube_drop_requested = self.request_cube_pose(*drop_pose_world)
            if self.state_elapsed() >= 0.8 and not self.pose_transport_busy():
                self.transition_to("ARM_HOME")
            elif self.state_elapsed() >= 2.0:
                self.get_logger().warn("Drop timed out; continuing mission")
                self.clear_pose_transport()
                self.transition_to("ARM_HOME")
            return

        if self.mission_state == "ARM_HOME":
            self.stop_robot()
            if self.entered_state():
                self.publish_right_arm_pose(RIGHT_ARM_HOME, use_action=True)
            elif self.should_refresh_arm_command():
                self.publish_right_arm_pose(RIGHT_ARM_HOME)
            if self.state_elapsed() >= ARM_MOVE_TIME:
                self.transition_to("DONE")

        if self.mission_state == "EXPLORE":
            if self.follow_active_route():
                self.transition_to("EXPLORE_DONE")
            return

        if self.mission_state == "EXPLORE_DONE":
            self.stop_robot()
            return

        if self.mission_state == "DONE":
            self.stop_robot()
            if self.entered_state():
                if self.start_explore_route():
                    self.transition_to("EXPLORE")
                else:
                    self.transition_to("EXPLORE_DONE")
            return

    def follow_active_route(self):
        if self.route_index >= len(self.active_route):
            self.stop_robot()
            return True

        gx, gy, gyaw = self.active_route[self.route_index]
        if self.navigate_to_pose(gx, gy, gyaw):
            self.get_logger().info(
                f"Reached waypoint {self.route_index} at ({gx:.2f}, {gy:.2f}, {gyaw:.2f})"
            )
            self.route_index += 1

        return self.route_index >= len(self.active_route)

    def navigate_to_pose(self, gx, gy, gyaw):
        distance, heading_error, final_yaw_error = self.pose_errors(gx, gy, gyaw)

        if distance <= self.position_tolerance:
            if abs(final_yaw_error) <= self.yaw_tolerance:
                self.stop_robot()
                return True

            angular = self.clamp(1.5 * final_yaw_error, -self.max_angular, self.max_angular)
            self.publish_cmd(0.0, angular)
            return False

        if abs(heading_error) > self.turn_only_threshold:
            angular = self.clamp(1.5 * heading_error, -self.max_angular, self.max_angular)
            self.publish_cmd(0.0, angular)
            return False

        linear = min(self.max_linear, 0.65 * distance)
        linear *= max(0.25, 1.0 - abs(heading_error))

        if self.front_min < self.nav_front_slow:
            linear = min(linear, 0.10)
        if self.front_min < self.nav_front_stop:
            linear = 0.0

        angular = self.clamp(1.2 * heading_error, -self.max_angular, self.max_angular)
        self.publish_cmd(linear, angular)
        return False

    def execute_backout(self):
        x, y, yaw = self.current_pose()

        if self.entered_state():
            self.backout_start_xy = (x, y)
            self.backout_hold_yaw = yaw
            self.get_logger().info("Starting reverse retreat with cube attached")

        progress = math.hypot(x - self.backout_start_xy[0], y - self.backout_start_xy[1])
        yaw_error = self.wrap_angle(self.backout_hold_yaw - yaw)

        if progress >= self.reverse_distance:
            self.stop_robot()
            return True

        if self.back_min <= self.reverse_back_stop:
            self.get_logger().warn("Back-out stopped early: obstacle detected behind robot")
            self.stop_robot()
            return True

        remaining = max(0.0, self.reverse_distance - progress)
        reverse_speed = min(self.reverse_linear_limit, 0.10 + 0.35 * remaining)

        if self.back_min < self.reverse_back_slow:
            reverse_speed = min(reverse_speed, 0.10)

        angular = self.clamp(1.6 * yaw_error, -0.80, 0.80)
        self.publish_cmd(-reverse_speed, angular)
        return False


    def publish_right_arm_pose(self, positions, use_action=False):
        self.right_arm_target = list(positions)

        traj = JointTrajectory()
        traj.joint_names = [
            "right_arm_shoulder_joint",
            "right_arm_elbow_joint",
            "right_arm_wrist_joint",
        ]

        point = JointTrajectoryPoint()
        point.positions = list(positions)
        seconds = int(ARM_MOVE_TIME)
        nanoseconds = int((ARM_MOVE_TIME - seconds) * 1_000_000_000)
        point.time_from_start = Duration(sec=seconds, nanosec=nanoseconds)
        traj.points = [point]

        self.right_arm_traj_pub.publish(traj)

        if use_action and self.right_arm_action.server_is_ready():
            goal = FollowJointTrajectory.Goal()
            goal.trajectory = traj
            self.right_arm_goal_future = self.right_arm_action.send_goal_async(goal)

    def should_refresh_arm_command(self):
        return int(self.state_elapsed() * 10.0) % 5 == 0

    def pose_transport_busy(self):
        return self.set_pose_future is not None and not self.set_pose_future.done()

    def clear_pose_transport(self):
        self.set_pose_future = None
        self.set_pose_request_started_ns = 0

    def request_cube_pose(self, wx, wy, wz, wyaw):
        if self.set_pose_future is not None and not self.set_pose_future.done():
            return False

        if self.set_pose_client is None or not self.set_pose_client.service_is_ready():
            now_ns = self.get_clock().now().nanoseconds
            if now_ns - self.last_pose_error_ns > 2_000_000_000:
                self.get_logger().warn(
                    f"Waiting for {self.set_pose_service_name} to become ready"
                )
                self.last_pose_error_ns = now_ns
            return False

        request = SetEntityPose.Request()
        request.entity.name = self.target_cube
        request.entity.type = Entity.MODEL
        request.pose.position.x = wx
        request.pose.position.y = wy
        request.pose.position.z = wz

        qx, qy, qz, qw = self.quaternion_from_yaw(wyaw)
        request.pose.orientation.x = qx
        request.pose.orientation.y = qy
        request.pose.orientation.z = qz
        request.pose.orientation.w = qw

        self.set_pose_future = self.set_pose_client.call_async(request)
        self.set_pose_request_started_ns = self.get_clock().now().nanoseconds
        return True

    def pump_set_pose_transport(self):
        if self.set_pose_future is None:
            return

        if self.set_pose_future.done():
            try:
                response = self.set_pose_future.result()
                if response is not None and not response.success:
                    self.get_logger().warn("SetEntityPose reported failure")
            except Exception as exc:
                now_ns = self.get_clock().now().nanoseconds
                if now_ns - self.last_pose_error_ns > 2_000_000_000:
                    self.get_logger().warn(f"SetEntityPose failed: {exc}")
                    self.last_pose_error_ns = now_ns
            self.clear_pose_transport()
            return

        elapsed = (
            self.get_clock().now().nanoseconds - self.set_pose_request_started_ns
        ) / 1e9
        if elapsed >= POSE_REQUEST_TIMEOUT:
            now_ns = self.get_clock().now().nanoseconds
            if now_ns - self.last_pose_error_ns > 2_000_000_000:
                self.get_logger().warn("SetEntityPose timed out; clearing request")
                self.last_pose_error_ns = now_ns
            self.clear_pose_transport()

    def update_attached_cube_pose(self, force=False):
        now_ns = self.get_clock().now().nanoseconds
        if not force and now_ns - self.last_cube_update_ns < 100_000_000:
            return

        magnet_x, magnet_y, magnet_z = self.right_magnet_world_position()
        _, _, robot_yaw = self.current_world_pose()
        cube_z = max(CUBE_HALF_HEIGHT, magnet_z - CUBE_HALF_HEIGHT)

        if self.request_cube_pose(magnet_x, magnet_y, cube_z, robot_yaw):
            self.last_cube_update_ns = now_ns

    def right_magnet_world_position(self):
        wx, wy, wyaw = self.current_world_pose()

        rx, ry, rz = self.right_magnet_base_position(self.right_arm_target)
        mx = wx + math.cos(wyaw) * rx - math.sin(wyaw) * ry
        my = wy + math.sin(wyaw) * rx + math.cos(wyaw) * ry
        mz = rz
        return mx, my, mz

    def right_magnet_base_position(self, joints):
        shoulder, elbow, wrist = joints

        rotation = self.rotz(-math.pi / 2.0)
        position = [0.0, RIGHT_ARM_MOUNT_Y, RIGHT_ARM_MOUNT_Z]

        self.translate_along_local_x(position, rotation, 0.025)
        rotation = self.mm(rotation, self.roty(-shoulder))
        self.translate_along_local_x(position, rotation, 0.22)
        rotation = self.mm(rotation, self.roty(-elbow))
        self.translate_along_local_x(position, rotation, 0.20)
        rotation = self.mm(rotation, self.roty(-wrist))
        self.translate_along_local_x(position, rotation, 0.12)

        return tuple(position)

    def translate_along_local_x(self, position, rotation, distance):
        offset = self.mv(rotation, (distance, 0.0, 0.0))
        position[0] += offset[0]
        position[1] += offset[1]
        position[2] += offset[2]

    def rotz(self, angle):
        c = math.cos(angle)
        s = math.sin(angle)
        return (
            (c, -s, 0.0),
            (s, c, 0.0),
            (0.0, 0.0, 1.0),
        )

    def roty(self, angle):
        c = math.cos(angle)
        s = math.sin(angle)
        return (
            (c, 0.0, s),
            (0.0, 1.0, 0.0),
            (-s, 0.0, c),
        )

    def mm(self, a, b):
        return tuple(
            tuple(sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3))
            for i in range(3)
        )

    def mv(self, matrix, vector):
        return tuple(sum(matrix[i][k] * vector[k] for k in range(3)) for i in range(3))

    def status_loop(self):
        self.get_logger().info(
            f"state={self.mission_state} cube={self.target_cube} "
            f"mode={self.mode} room={self.current_room} rooms={sorted(self.rooms_visited)} "
            f"route={self.active_route_name} {self.route_index}/{len(self.active_route)} "
            f"cube_attached={self.cube_attached} dist={self.distance_traveled:.2f} "
            f"front={self.front_min:.2f} back={self.back_min:.2f} "
            f"left={self.left_side_min:.2f} right={self.right_side_min:.2f} "
            f"front_left={self.front_left_min:.2f} front_right={self.front_right_min:.2f} "
            f"turn_dir={'left' if self.turn_dir > 0 else 'right'} "
        )

    def publish_cmd(self, linear_x, angular_z):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = linear_x
        msg.twist.angular.z = angular_z
        self.cmd_pub.publish(msg)

    def stop_robot(self):
        self.publish_cmd(0.0, 0.0)

    def sector_min(self, scan, deg_min, deg_max, min_valid=0.0):
        vals = []
        angle_start = math.degrees(scan.angle_min)
        angle_increment = math.degrees(scan.angle_increment)

        for i, reading in enumerate(scan.ranges):
            if not math.isfinite(reading) or reading < min_valid:
                continue

            angle = angle_start + i * angle_increment
            if deg_min <= deg_max:
                in_sector = deg_min <= angle <= deg_max
            else:
                in_sector = angle >= deg_min or angle <= deg_max

            if in_sector:
                vals.append(reading)

        return min(vals) if vals else float("inf")

    def classify_room(self, y):
        if y > 0.8:
            return "north"
        if y < -0.8:
            return "south"
        return "center"

    def destroy_node(self):
        try:
            self.odom_file.flush()
            self.odom_file.close()
            self.save_path_png()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WaypointNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
