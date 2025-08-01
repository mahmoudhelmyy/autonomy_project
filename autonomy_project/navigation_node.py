"""
Navigation control node for the Moebius challenge.

This module implements the high‑level autonomy required to complete
three laps of the square arena while hugging the inner wall and
avoiding coloured obstacles.  It uses LaserScan data from the
LDROBOT LD500 lidar and odometry feedback to determine
direction (clockwise or counter‑clockwise), count laps, and perform
wall following using a simple PID controller.  Obstacle avoidance
logic subscribes to the custom ``autonomy_project/Obstacle`` topic
published by the obstacle detector and triggers a timed manoeuvre
that steers around red and green blocks.

Key assumptions:

* The lidar node publishes scans on ``/scan_raw``.  The official
  LDROBOT ROS 2 driver exposes a ``/ldlidar_node/scan`` LaserScan
  topic once activated【994921289684312†L392-L401】.  You should remap the
  driver's topic to ``/scan_raw`` in your launch file or adjust
  ``self.scan_sub`` below.
* The chassis controller subscribes to ``/controller/cmd_vel`` for
  ``geometry_msgs/Twist`` commands.  Adjust the topic name if your
  controller uses a different interface.
* Odometry feedback is available on ``/odom`` and provides
  orientation as a quaternion.  The method ``get_yaw_from_pose``
  extracts the yaw (heading) from this quaternion.
"""

import math
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from autonomy_project.msg import Obstacle


class PIDController:
    """Simple proportional–integral–derivative controller.

    This class encapsulates a basic PID control loop.  Only the
    position form is implemented, as it suffices for wall following.
    """

    def __init__(self, kp: float, ki: float, kd: float, setpoint: float) -> None:
        self.kp, self.ki, self.kd = kp, ki, kd
        self.setpoint = setpoint
        self.prev_error = 0.0
        self.integral = 0.0

    def calculate(self, current_value: float, dt: float) -> float:
        error = self.setpoint - current_value
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        self.prev_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative


class NavigationNode(Node):
    """Main state machine for trajectory following and obstacle avoidance."""

    def __init__(self) -> None:
        super().__init__('navigation_node')

        # --- Tunable parameters (may be exposed as ROS parameters) ---
        # Forward speed when following the wall [m/s]
        self.linear_speed: float = 0.22
        # Desired distance from inner wall [m]
        self.desired_wall_dist: float = 0.35
        # Threshold distance to an obstacle before avoidance manoeuvre [m]
        self.obstacle_dist_thresh: float = 0.7
        # PID gains for wall following
        self.pid_kp: float = 1.8
        self.pid_ki: float = 0.0
        self.pid_kd: float = 0.6

        # --- Internal state for lap counting and manoeuvres ---
        self.state: str = 'INITIALIZING'
        self.direction: Optional[str] = None  # 'CW' or 'CCW'
        self.start_pose: Optional[Odometry] = None
        self.last_yaw: Optional[float] = None
        self.lap_count: int = 0
        self.total_yaw: float = 0.0
        self.maneuver_start_time = None  # type: Optional[rclpy.time.Time]
        self.maneuver_state: int = 0

        # PID controller instance
        self.pid = PIDController(self.pid_kp, self.pid_ki, self.pid_kd, self.desired_wall_dist)

        # Publishers and subscribers
        self.cmd_pub = self.create_publisher(Twist, '/controller/cmd_vel', 10)
        # See note above about remapping the scan topic
        self.scan_sub = self.create_subscription(LaserScan, '/scan_raw', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.obstacle_sub = self.create_subscription(Obstacle, '/obstacles', self.obstacle_callback, 10)

        self.get_logger().info('NavigationNode started')

    # ---------------------------------------------------------------------
    # Callback functions
    # ---------------------------------------------------------------------
    def odom_callback(self, msg: Odometry) -> None:
        """Handle odometry updates for lap counting and yaw tracking."""
        if self.start_pose is None:
            # On first callback record the starting pose and yaw
            self.start_pose = msg.pose.pose
            self.last_yaw = self.get_yaw_from_pose(self.start_pose)
            return
        current_yaw = self.get_yaw_from_pose(msg.pose.pose)
        dyaw = current_yaw - self.last_yaw
        # Normalize angle difference to [-pi, pi]
        if dyaw > np.pi:
            dyaw -= 2 * np.pi
        if dyaw < -np.pi:
            dyaw += 2 * np.pi
        self.total_yaw += dyaw
        self.last_yaw = current_yaw
        # Check for lap completion when returning close to start
        lap_threshold = 1.95 * np.pi  # slightly less than 2π to account for noise
        start_dist = np.hypot(
            msg.pose.pose.position.x - self.start_pose.position.x,
            msg.pose.pose.position.y - self.start_pose.position.y,
        )
        if (
            self.direction == 'CW' and self.total_yaw < -lap_threshold and start_dist < 0.5
        ) or (
            self.direction == 'CCW' and self.total_yaw > lap_threshold and start_dist < 0.5
        ):
            self.lap_count += 1
            self.total_yaw = 0.0
            self.get_logger().info(f'Lap {self.lap_count} completed')

    def obstacle_callback(self, msg: Obstacle) -> None:
        """Trigger avoidance when an obstacle is within threshold distance."""
        if self.state == 'FOLLOWING_WALL' and msg.distance < self.obstacle_dist_thresh:
            # Decide which direction to turn based on colour: red -> right, green -> left
            self.get_logger().info(f'Obstacle detected: {msg.color}, initiating avoidance manoeuvre')
            self.state = 'AVOIDING_RIGHT' if msg.color == 'red' else 'AVOIDING_LEFT'
            self.maneuver_state = 0

    def scan_callback(self, msg: LaserScan) -> None:
        """Main loop driven by LaserScan data."""
        if self.lap_count >= 3:
            # Stop the robot after completing three laps
            if self.state != 'FINISHED':
                self.get_logger().info('Mission complete: three laps finished')
                self.state = 'FINISHED'
            self.publish_twist(0.0, 0.0)
            return
        # State machine
        if self.state == 'INITIALIZING':
            self.initialize_direction(msg)
        elif self.state == 'FOLLOWING_WALL':
            self.follow_wall(msg)
        elif self.state == 'AVOIDING_RIGHT':
            self.avoid_maneuver(-0.7)  # Negative angular velocity turns right
        elif self.state == 'AVOIDING_LEFT':
            self.avoid_maneuver(0.7)   # Positive turns left

    # ---------------------------------------------------------------------
    # State machine helper functions
    # ---------------------------------------------------------------------
    def initialize_direction(self, scan_msg: LaserScan) -> None:
        """Determine clockwise vs counter‑clockwise direction at start.

        This method compares the minimum range to the left and right
        quarter sectors of the scan to decide which side of the arena is
        closer.  The robot then follows the inner wall in the direction
        that keeps the larger gap inside the arena (shortest path).
        """
        ranges = np.array(scan_msg.ranges)
        # Only consider finite range readings
        right_view = ranges[int(len(ranges) * 0.75) :]
        left_view = ranges[: int(len(ranges) * 0.25)]
        min_right = np.min(right_view[np.isfinite(right_view)])
        min_left = np.min(left_view[np.isfinite(left_view)])
        self.direction = 'CW' if min_right > min_left else 'CCW'
        self.state = 'FOLLOWING_WALL'
        self.get_logger().info(f'Direction determined: {self.direction}')

    def follow_wall(self, scan_msg: LaserScan) -> None:
        """Follow the inner wall using a PID controller."""
        ranges = np.array(scan_msg.ranges)
        # Compute indices corresponding to a 60° slice either on the left or right
        if self.direction == 'CW':
            angle_min_idx = int(np.deg2rad(240) / scan_msg.angle_increment)
            angle_max_idx = int(np.deg2rad(300) / scan_msg.angle_increment)
        else:
            angle_min_idx = int(np.deg2rad(60) / scan_msg.angle_increment)
            angle_max_idx = int(np.deg2rad(120) / scan_msg.angle_increment)
        wall_slice = ranges[angle_min_idx:angle_max_idx]
        wall_slice = wall_slice[np.isfinite(wall_slice)]
        if len(wall_slice) == 0:
            return
        # Use the minimum range as the current distance to the wall
        current_dist = float(np.min(wall_slice))
        # PID calculates the angular velocity correction
        angular_z = self.pid.calculate(current_dist, dt=0.1)  # dt ≈ 0.1 s
        if self.direction == 'CW':
            angular_z *= -1.0
        # Publish forward velocity and steering command
        self.publish_twist(self.linear_speed, angular_z)

    def avoid_maneuver(self, turn_angular_z: float) -> None:
        """Perform a timed manoeuvre to drive around an obstacle."""
        current_time = self.get_clock().now()
        if self.maneuver_state == 0:
            # Begin by turning around the obstacle
            self.publish_twist(0.1, turn_angular_z)
            self.maneuver_start_time = current_time
            self.maneuver_state = 1
        elif self.maneuver_state == 1:
            # Wait 1.5 s of turning
            if (current_time - self.maneuver_start_time).nanoseconds > 1.5e9:
                self.maneuver_state = 2
        elif self.maneuver_state == 2:
            # Drive straight for 2 s
            self.publish_twist(self.linear_speed, 0.0)
            self.maneuver_start_time = current_time
            self.maneuver_state = 3
        elif self.maneuver_state == 3:
            if (current_time - self.maneuver_start_time).nanoseconds > 2.0e9:
                self.maneuver_state = 4
        elif self.maneuver_state == 4:
            # Turn back to align with the wall again
            self.publish_twist(0.1, -turn_angular_z)
            self.maneuver_start_time = current_time
            self.maneuver_state = 5
        elif self.maneuver_state == 5:
            if (current_time - self.maneuver_start_time).nanoseconds > 1.5e9:
                self.get_logger().info('Avoidance complete, resuming wall following')
                self.state = 'FOLLOWING_WALL'

    # ---------------------------------------------------------------------
    # Utility functions
    # ---------------------------------------------------------------------
    def publish_twist(self, linear: float, angular: float) -> None:
        """Send a Twist command to the chassis controller."""
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = float(angular)
        self.cmd_pub.publish(msg)

    @staticmethod
    def get_yaw_from_pose(pose) -> float:
        """Extract yaw angle from a quaternion pose."""
        q = pose.orientation
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )


def main(args=None) -> None:
    """Entry point for the navigation node."""
    rclpy.init(args=args)
    node = NavigationNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()