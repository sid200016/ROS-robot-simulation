#!/usr/bin/env python3
"""TidyBot Behavior Node — autonomous home tidying task.

Navigates through two rooms using hard-coded waypoints, logs odometry,
publishes path data, and (bonus) attempts object pickup via arm control.
"""

import math
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import numpy as np


class TidyBotBehavior(Node):
    """Autonomous tidying behavior for TidyBot."""

    # ------------------------------------------------------------------ #
    #  Waypoints: (x, y, heading_deg)                                      #
    #  Room 1 (living room): x in [-7, 1], y in [-4, 4]                   #
    #  Room 2 (bedroom):     x in [ 1, 8], y in [-4, 4]                   #
    #  Robot starts at (-6, 0) facing +x                                   #
    # ------------------------------------------------------------------ #
    WAYPOINTS = [
        # Explore Room 1
        (-5.0,  2.0,   0.0),
        (-3.0,  2.0,   0.0),
        (-3.0, -2.0, 180.0),
        (-5.0, -2.0, 180.0),
        (-6.0,  0.0,   0.0),
        # Pass through doorway into Room 2
        ( 0.0,  0.0,   0.0),
        ( 2.0,  0.0,   0.0),
        # Explore Room 2
        ( 4.0,  2.0,   0.0),
        ( 6.0,  2.0,   0.0),
        ( 6.0, -2.0, 180.0),
        ( 4.0, -2.0, 180.0),
        ( 2.0,  0.0,   0.0),
        # Return to Room 1 / collection box
        (-1.0,  0.0, 180.0),
        (-5.0,  0.0, 180.0),
    ]

    # Known object positions (x, y) — from world file placement
    OBJECT_POSITIONS = [
        (-4.0,  1.5),   # red block Room 1
        (-2.5, -1.5),   # blue cylinder Room 1
        (-6.0,  2.5),   # green box Room 1
        ( 3.0,  1.5),   # yellow block Room 2
        ( 5.5, -1.5),   # orange cylinder Room 2
        ( 4.0,  2.5),   # purple box Room 2
    ]

    # Collection box position
    COLLECTION_BOX = (-5.5, -0.5)

    # Control parameters
    LINEAR_SPEED   = 0.3   # m/s
    ANGULAR_SPEED  = 0.6   # rad/s
    GOAL_TOLERANCE = 0.25  # m
    ANGLE_TOLERANCE = 0.08 # rad

    def __init__(self):
        super().__init__('tidybot_behavior')

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub  = self.create_publisher(String, '/tidybot/status', 10)
        self.arm_pub     = self.create_publisher(
            JointTrajectory, '/arm_controller/joint_trajectory', 10)

        # Subscribers
        self.odom_sub  = self.create_subscription(
            Odometry, '/odom', self._odom_cb, 10)
        self.scan_sub  = self.create_subscription(
            LaserScan, '/scan', self._scan_cb, 10)
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self._image_cb, 10)

        # State
        self.x = -6.0
        self.y =  0.0
        self.yaw = 0.0
        self.odom_received = False
        self.scan_data = None
        self.image_count = 0

        # Task state
        self.waypoint_idx = 0
        self.state = 'NAVIGATE'   # NAVIGATE | PICKUP | RETURN | DONE
        self.collected_objects = []
        self.total_distance = 0.0
        self.prev_x = self.x
        self.prev_y = self.y
        self.rooms_visited = set()
        self.start_time = self.get_clock().now()

        # Main control loop at 10 Hz
        self.timer = self.create_timer(0.1, self._control_loop)

        self.get_logger().info('TidyBot behavior node started.')
        self.get_logger().info(
            f'Navigating {len(self.WAYPOINTS)} waypoints across 2 rooms.')

    # ------------------------------------------------------------------ #
    #  Callbacks                                                           #
    # ------------------------------------------------------------------ #
    def _odom_cb(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        # yaw from quaternion
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)
        self.odom_received = True

        # Accumulate distance
        dx = self.x - self.prev_x
        dy = self.y - self.prev_y
        self.total_distance += math.sqrt(dx*dx + dy*dy)
        self.prev_x = self.x
        self.prev_y = self.y

        # Track rooms
        if self.x < 1.0:
            self.rooms_visited.add('Room1_LivingRoom')
        else:
            self.rooms_visited.add('Room2_Bedroom')

    def _scan_cb(self, msg: LaserScan):
        self.scan_data = msg

    def _image_cb(self, msg: Image):
        self.image_count += 1
        if self.image_count % 100 == 0:
            self.get_logger().info(
                f'Camera active — {self.image_count} frames received.')

    # ------------------------------------------------------------------ #
    #  Helpers                                                             #
    # ------------------------------------------------------------------ #
    def _angle_diff(self, a: float, b: float) -> float:
        """Signed difference a - b, wrapped to [-pi, pi]."""
        d = a - b
        while d >  math.pi: d -= 2*math.pi
        while d < -math.pi: d += 2*math.pi
        return d

    def _dist_to(self, tx: float, ty: float) -> float:
        return math.sqrt((tx - self.x)**2 + (ty - self.y)**2)

    def _heading_to(self, tx: float, ty: float) -> float:
        return math.atan2(ty - self.y, tx - self.x)

    def _obstacle_ahead(self) -> bool:
        """Return True if LiDAR detects obstacle within 0.4 m in front."""
        if self.scan_data is None:
            return False
        ranges = self.scan_data.ranges
        n = len(ranges)
        if n == 0:
            return False
        # Check ±20° around front (index 0)
        arc = int(20 * n / 360)
        front = ranges[:arc] + ranges[n-arc:]
        valid = [r for r in front if 0.05 < r < 0.4]
        return len(valid) > 3

    def _publish_status(self, msg: str):
        s = String()
        s.data = msg
        self.status_pub.publish(s)
        self.get_logger().info(msg)

    def _stop(self):
        self.cmd_vel_pub.publish(Twist())

    def _move_to(self, tx: float, ty: float) -> bool:
        """Drive toward (tx, ty). Returns True when goal reached."""
        dist = self._dist_to(tx, ty)
        if dist < self.GOAL_TOLERANCE:
            self._stop()
            return True

        desired_heading = self._heading_to(tx, ty)
        heading_err = self._angle_diff(desired_heading, self.yaw)

        twist = Twist()
        if abs(heading_err) > self.ANGLE_TOLERANCE:
            # Rotate in place first
            twist.angular.z = self.ANGULAR_SPEED * (1.0 if heading_err > 0 else -1.0)
        else:
            # Drive forward
            twist.linear.x  = min(self.LINEAR_SPEED, dist * 0.8)
            twist.angular.z = 1.5 * heading_err  # proportional correction

        # Obstacle avoidance: turn right if blocked
        if self._obstacle_ahead():
            twist.linear.x  = 0.0
            twist.angular.z = -self.ANGULAR_SPEED

        self.cmd_vel_pub.publish(twist)
        return False

    def _send_arm_pose(self, shoulder: float, elbow: float, wrist: float):
        """Send a joint trajectory command to the arm."""
        traj = JointTrajectory()
        traj.joint_names = [
            'right_shoulder_joint',
            'right_elbow_joint',
            'right_wrist_joint',
        ]
        pt = JointTrajectoryPoint()
        pt.positions = [shoulder, elbow, wrist]
        pt.time_from_start = Duration(sec=2, nanosec=0)
        traj.points = [pt]
        self.arm_pub.publish(traj)

    def _arm_reach_down(self):
        """Move arm to reach-down (pickup) pose."""
        self._send_arm_pose(
            shoulder=1.2,   # ~70° forward
            elbow=1.0,      # ~57° down
            wrist=-0.5,
        )

    def _arm_carry(self):
        """Move arm to carry pose (object held up)."""
        self._send_arm_pose(
            shoulder=0.3,
            elbow=-0.5,
            wrist=0.0,
        )

    def _arm_home(self):
        """Return arm to home/stowed pose."""
        self._send_arm_pose(0.0, 0.0, 0.0)

    # ------------------------------------------------------------------ #
    #  Main control loop                                                   #
    # ------------------------------------------------------------------ #
    def _control_loop(self):
        if not self.odom_received:
            return

        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9

        # ---- DONE ----
        if self.state == 'DONE':
            self._stop()
            return

        # ---- Safety timeout (4.5 min sim time) ----
        if elapsed > 270.0:
            self._publish_status(
                f'[TIMEOUT] Stopping after {elapsed:.1f}s. '
                f'Distance={self.total_distance:.2f}m, '
                f'Rooms={self.rooms_visited}, '
                f'Objects collected={len(self.collected_objects)}')
            self._stop()
            self.state = 'DONE'
            return

        # ---- NAVIGATE ----
        if self.state == 'NAVIGATE':
            if self.waypoint_idx >= len(self.WAYPOINTS):
                self._publish_status(
                    f'[COMPLETE] All waypoints visited. '
                    f'Distance={self.total_distance:.2f}m, '
                    f'Rooms={self.rooms_visited}')
                self.state = 'DONE'
                return

            wx, wy, _ = self.WAYPOINTS[self.waypoint_idx]
            reached = self._move_to(wx, wy)

            if reached:
                self._publish_status(
                    f'[WP {self.waypoint_idx+1}/{len(self.WAYPOINTS)}] '
                    f'Reached ({wx:.1f}, {wy:.1f}). '
                    f'Dist={self.total_distance:.2f}m')
                self.waypoint_idx += 1

                # Check if near a known object
                for obj_pos in self.OBJECT_POSITIONS:
                    if obj_pos not in self.collected_objects:
                        if self._dist_to(*obj_pos) < 1.0:
                            self._publish_status(
                                f'[OBJECT] Detected object near '
                                f'({obj_pos[0]:.1f}, {obj_pos[1]:.1f})')
                            self.target_object = obj_pos
                            self.state = 'PICKUP'
                            self._pickup_phase = 'APPROACH'
                            return

        # ---- PICKUP ----
        elif self.state == 'PICKUP':
            ox, oy = self.target_object

            if self._pickup_phase == 'APPROACH':
                reached = self._move_to(ox, oy)
                if reached:
                    self._publish_status(
                        f'[PICKUP] At object ({ox:.1f}, {oy:.1f}). '
                        f'Lowering arm...')
                    self._arm_reach_down()
                    self._pickup_phase = 'GRAB'
                    self._phase_timer = elapsed

            elif self._pickup_phase == 'GRAB':
                # Wait 2 s for arm to reach down
                if elapsed - self._phase_timer > 2.0:
                    self._publish_status(
                        f'[PICKUP] Grasped object at '
                        f'({ox:.1f}, {oy:.1f}). Carrying...')
                    self._arm_carry()
                    self.collected_objects.append(self.target_object)
                    self._pickup_phase = 'RETURN'
                    self._phase_timer = elapsed

            elif self._pickup_phase == 'RETURN':
                # Wait 1 s then navigate to collection box
                if elapsed - self._phase_timer > 1.0:
                    bx, by = self.COLLECTION_BOX
                    reached = self._move_to(bx, by)
                    if reached:
                        self._publish_status(
                            f'[DEPOSIT] Deposited object in collection box. '
                            f'Total collected: {len(self.collected_objects)}')
                        self._arm_home()
                        self.state = 'NAVIGATE'


def main(args=None):
    rclpy.init(args=args)
    node = TidyBotBehavior()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(
            f'[SHUTDOWN] Total distance: {node.total_distance:.2f}m, '
            f'Rooms visited: {node.rooms_visited}, '
            f'Objects collected: {len(node.collected_objects)}')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
