#!/usr/bin/env python3

import math
import random

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped, PoseStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
import csv
from datetime import datetime
from pathlib import Path as pt

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from rclpy.executors import ExternalShutdownException
WORLD_START = (-3.0, 6.0, 0.0)
BOX_WORLD = (-1.5, 7.5)
BOX_APPROACH_WORLD = (-1.5, 6.8)

KNOWN_CUBES = {
    "obs1": (0.5, 0.0),
    "obs2": (3.0, 2.5),
    "obs3": (3.0, -2.5),
    "obs4": (-3.0, 1.5),
    "obs5": (-3.0, -1.5),   
    "obs6": (3.0, 6.0),
}

DROP_SLOTS = [
    (0.00, 0.00),
    (0.08, 0.00),
    (-0.08, 0.00),
    (0.00, 0.08),
    (0.00, -0.08),
]


class ExplorerNode(Node):
    def __init__(self):
        super().__init__("explorer_node")

        self.scan_msg = None
        self.odom_msg = None

        self.front_min = float("inf")
        self.front_left_min = float("inf")
        self.front_right_min = float("inf")
        self.left_side_min = float("inf")
        self.right_side_min = float("inf")
        self.back_min = float("inf")
        self.self_side_ignore = 0.35

        self.front_stop = 0.9
        self.front_corner_stop = 0.9
        self.corner_caution = 0.9

        self.side_drive_stop = 0.50
        self.side_turn_stop = 0.55
        self.back_stop = 0.55


        self.mode = "FORWARD"
        self.turn_dir = 1.0

        self.distance_traveled = 0.0
        self.prev_xy = None
        self.rooms_visited = set()

        self.scan_sub = self.create_subscription(LaserScan, "/scan", self.scan_cb, 10)
        self.odom_sub = self.create_subscription(
            Odometry, "/odometry/filtered", self.odom_cb, 10
        )

        self.cmd_pub = self.create_publisher(
            TwistStamped, "/diff_drive_controller/cmd_vel", 10
        )
        self.path_pub = self.create_publisher(Path, "/path", 10)

        self.path_msg = Path()
        self.path_msg.header.frame_id = "odom"

        self.control_timer = self.create_timer(0.01, self.control_loop)
        self.status_timer = self.create_timer(1.0, self.status_loop)
        workspace_root = pt(__file__).resolve().parents[4]
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.run_dir = workspace_root / "output" / stamp
        self.run_dir.mkdir(parents=True, exist_ok=True)

        self.odom_file = (self.run_dir / "odom.csv").open("w", newline="")
        self.odom_writer = csv.writer(self.odom_file)
        self.odom_writer.writerow(["t", "x", "y", "yaw", "mode", "front", "front_left", "front_right"])

        self.png_timer = self.create_timer(2.0, self.save_path_png) 
        self.odom_flush_counter = 0


    def scan_cb(self, msg):
        self.scan_msg = msg
        self.front_min       = self.sector_min(msg, -25.0,  25.0)
        self.front_left_min  = self.sector_min(msg,  25.0,  70.0)
        self.front_right_min = self.sector_min(msg, -70.0, -25.0)
        self.left_side_min   = self.sector_min(msg,  70.0, 150.0, self.self_side_ignore)
        self.right_side_min  = self.sector_min(msg, -150.0, -70.0, self.self_side_ignore)
        self.back_min        = self.sector_min(msg, 150.0, -150.0)

    def yaw_from_quat(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)
    def save_path_png(self):
        if not self.path_msg.poses:
            return

        xs = [p.pose.position.x for p in self.path_msg.poses]
        ys = [p.pose.position.y for p in self.path_msg.poses]

        fig, ax = plt.subplots(figsize=(6, 10))
        ax.plot(xs, ys, linewidth=2)
        ax.scatter(xs[0], ys[0], c="green", s=40)
        ax.scatter(xs[-1], ys[-1], c="red", s=40)
        ax.set_xlabel("x (m)")
        ax.set_ylabel("y (m)")
        ax.set_title("Robot Path")
        ax.grid(True)
        ax.axis("equal")
        fig.tight_layout()
        fig.savefig(self.run_dir / "path_live.png", dpi=180)
        plt.close(fig)


    def odom_cb(self, msg):
        self.odom_msg = msg

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        if self.prev_xy is not None:
            self.distance_traveled += math.hypot(x - self.prev_xy[0], y - self.prev_xy[1])
        self.prev_xy = (x, y)

        if y > 0.8:
            self.rooms_visited.add("north")
        elif y < -0.8:
            self.rooms_visited.add("south")

        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.path_msg.header.stamp = msg.header.stamp
        self.path_msg.poses.append(pose)
        self.path_pub.publish(self.path_msg)
        yaw = self.yaw_from_quat(msg.pose.pose.orientation)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        self.odom_writer.writerow([
            stamp, x, y, yaw, self.mode,
            self.front_min, self.front_left_min, self.front_right_min
        ])

        self.odom_flush_counter += 1
        if self.odom_flush_counter >= 10:
            self.odom_file.flush()
            self.odom_flush_counter = 0


    def control_loop(self):
        if self.scan_msg is None or self.odom_msg is None:
            self.stop_robot()
            return
        blocked_ahead = (
            self.front_min < self.front_stop or
            self.front_left_min < self.front_corner_stop or
            self.front_right_min < self.front_corner_stop
        )

        too_close_on_side = (
            self.left_side_min < self.side_drive_stop or
            self.right_side_min < self.side_drive_stop
        )
        
        if self.mode == "FORWARD":
            if blocked_ahead or too_close_on_side:
                self.mode = "TURN"
                left_score = min(self.front_left_min, self.left_side_min)
                right_score = min(self.front_right_min, self.right_side_min)
                self.turn_dir = 1.0 if left_score > right_score else -1.0

            elif self.front_left_min < self.front_corner_stop:
                self.publish_cmd(0.04, -0.6)   
            elif self.front_right_min < self.front_corner_stop:
                self.publish_cmd(0.04, 0.6)  

            elif self.front_left_min < self.corner_caution or self.front_right_min < self.corner_caution:
                if self.front_left_min < self.front_right_min:
                    self.publish_cmd(0.08, -0.25)
                else:
                    self.publish_cmd(0.08, 0.25)
            else:
                ang = random.uniform(-0.12, 0.12)
                self.publish_cmd(1.0, ang)

        elif self.mode == "TURN":
            turn_side_min = self.left_side_min if self.turn_dir > 0 else self.right_side_min
            if turn_side_min < self.side_turn_stop:
                if self.back_min > self.back_stop:
                    self.mode = "BACK_UP"
                    
                else:
                    self.turn_dir *= -1.0
            elif self.front_min > 0.9 and self.front_left_min > 0.7 and self.front_right_min > 0.7:
                self.mode = "FORWARD"
            else:
                self.publish_cmd(0.0, 0.25 * self.turn_dir)
        elif self.mode == "BACK_UP":
            if self.back_min < self.back_stop:
                self.mode = "TURN"
                
            else:
                self.publish_cmd(-0.15, 0.0)
    def status_loop(self):
        self.get_logger().info(
            f"mode={self.mode} front={self.front_min:.2f} "
            f"rooms={sorted(self.rooms_visited)} "
            f"dist={self.distance_traveled:.2f}"
            f"left={self.left_side_min:.2f} right={self.right_side_min:.2f} "
            f"back={self.back_min:.2f}" 
            f"turn_dir={'left' if self.turn_dir > 0 else 'right'}"
            f"blocked_ahead={self.front_min < self.front_stop} "
            f"too_close_on_side={self.left_side_min < self.side_drive_stop or self.right_side_min < self.side_drive_stop}"
            f"front_left_min={self.front_left_min:.2f} front_right_min={self.front_right_min:.2f}"
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
        a0 = math.degrees(scan.angle_min)
        inc = math.degrees(scan.angle_increment)

        for i, r in enumerate(scan.ranges):
            if not math.isfinite(r) or r < min_valid:
                continue
            angle = a0 + i * inc
            if deg_min <= deg_max:
                in_sector = deg_min <= angle <= deg_max
            else:
                in_sector = angle >= deg_min or angle <= deg_max
            if in_sector:
                vals.append(r)
        return min(vals) if vals else float("inf")
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
    node = ExplorerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
