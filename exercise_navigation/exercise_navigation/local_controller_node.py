#!/usr/bin/env python3
"""
Very simple local path follower (Pure Pursuit style) + simple laser stop.

Subscribes:
  /plan   (nav_msgs/Path)
  /scan   (sensor_msgs/LaserScan)

Publishes:
  /cmd_vel (geometry_msgs/Twist)

TF:
  needs transform map -> base_link (or your base_frame)

Behavior:
- If no path: publish 0.
- If obstacle too close in front: publish 0.
- Otherwise: follow a lookahead point on the path:
    angular.z proportional to heading error
    linear.x constant (slows down for big turns)
- Stops when goal reached.

This is meant for teaching / labs.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan

import tf2_ros


def yaw_from_quaternion(q):
    # q has fields x,y,z,w
    # yaw = atan2(2(wz + xy), 1 - 2(y^2 + z^2))
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_to_pi(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class LocalPathFollower(Node):
    def __init__(self):
        super().__init__("local_path_follower")

        # ---- Parameters (easy knobs) ----
        self.declare_parameter("plan_topic", "/plan")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")

        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("tf_timeout_sec", 0.2)

        self.declare_parameter("control_rate_hz", 20.0)

        self.declare_parameter("lookahead_dist", 0.20)   # meters
        self.declare_parameter("linear_speed", 0.25)     # m/s (max)
        self.declare_parameter("max_angular", 1.5)       # rad/s
        self.declare_parameter("k_ang", 1.8)             # angular gain

        self.declare_parameter("goal_tolerance", 0.20)   # meters

        # Simple scan stop
        self.declare_parameter("use_scan_stop", True)
        self.declare_parameter("stop_dist", 0.35)        # meters
        self.declare_parameter("front_angle_deg", 25.0)  # degrees to each side

        # ---- ROS I/O ----
        self.plan_topic = self.get_parameter("plan_topic").value
        self.scan_topic = self.get_parameter("scan_topic").value
        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.plan_sub = self.create_subscription(Path, self.plan_topic, self.on_plan, 10)
        self.scan_sub = self.create_subscription(LaserScan, self.scan_topic, self.on_scan, 10)

        # ---- TF ----
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ---- State ----
        self.path = None
        self.have_path = False
        self.closest_index = 0

        self.scan_min_front = None

        # ---- Control loop ----
        rate = float(self.get_parameter("control_rate_hz").value)
        self.timer = self.create_timer(1.0 / rate, self.control_step)

        self.get_logger().info("LocalPathFollower started. Waiting for /plan...")

    def on_plan(self, msg):
        if len(msg.poses) == 0:
            self.get_logger().warn("Received empty path.")
            self.path = None
            self.have_path = False
            self.closest_index = 0
            return

        self.path = msg
        self.have_path = True
        self.closest_index = 0
        self.get_logger().info(f"Received path with {len(msg.poses)} poses.")

    def on_scan(self, msg):
        if not bool(self.get_parameter("use_scan_stop").value):
            self.scan_min_front = None
            return

        front_deg = float(self.get_parameter("front_angle_deg").value)
        front_rad = math.radians(front_deg)

        # Find indices for angles in [-front_rad, +front_rad]
        # LaserScan angles: angle_min + i * angle_increment
        i_min = int(round(((-front_rad) - msg.angle_min) / msg.angle_increment))
        i_max = int(round(((+front_rad) - msg.angle_min) / msg.angle_increment))
        i_min = max(0, i_min)
        i_max = min(len(msg.ranges) - 1, i_max)

        m = None
        for i in range(i_min, i_max + 1):
            r = msg.ranges[i]
            if math.isfinite(r) and r > 0.0:
                if m is None or r < m:
                    m = r

        self.scan_min_front = m  # could be None if no valid ranges

    def get_robot_pose_in_map(self):
        map_frame = self.get_parameter("map_frame").value
        base_frame = self.get_parameter("base_frame").value
        timeout_sec = float(self.get_parameter("tf_timeout_sec").value)

        try:
            tf = self.tf_buffer.lookup_transform(
                target_frame=map_frame,
                source_frame=base_frame,
                time=rclpy.time.Time(),
                timeout=Duration(seconds=timeout_sec),
            )
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed ({map_frame} <- {base_frame}): {e}")
            return None

        x = tf.transform.translation.x
        y = tf.transform.translation.y
        yaw = yaw_from_quaternion(tf.transform.rotation)
        return (x, y, yaw)

    def publish_cmd(self, vx, wz):
        msg = Twist()
        msg.linear.x = float(vx)
        msg.angular.z = float(wz)
        self.cmd_pub.publish(msg)

    def control_step(self):
        # Safety stop if no path
        if not self.have_path or self.path is None:
            self.publish_cmd(0.0, 0.0)
            return

        # Simple obstacle stop
        if bool(self.get_parameter("use_scan_stop").value):
            stop_dist = float(self.get_parameter("stop_dist").value)
            if self.scan_min_front is not None and self.scan_min_front < stop_dist:
                self.publish_cmd(0.0, 0.0)
                return

        pose = self.get_robot_pose_in_map()
        if pose is None:
            self.publish_cmd(0.0, 0.0)
            return

        rx, ry, ryaw = pose

        # Check goal reached (distance to final pose)
        goal = self.path.poses[-1].pose.position
        goal_dist = math.hypot(goal.x - rx, goal.y - ry)
        if goal_dist < float(self.get_parameter("goal_tolerance").value):
            self.publish_cmd(0.0, 0.0)
            return

        # 1) Find closest path point (starting from previous closest_index for efficiency)
        pts = self.path.poses
        start_i = self.closest_index
        best_i = start_i
        best_d2 = None

        # Search a window forward (keeps it simple and stable)
        window = 50
        end_i = min(len(pts), start_i + window)

        for i in range(start_i, end_i):
            px = pts[i].pose.position.x
            py = pts[i].pose.position.y
            d2 = (px - rx) * (px - rx) + (py - ry) * (py - ry)
            if best_d2 is None or d2 < best_d2:
                best_d2 = d2
                best_i = i

        self.closest_index = best_i

        # 2) Pick a lookahead point ahead of closest index
        lookahead = float(self.get_parameter("lookahead_dist").value)
        target_i = best_i
        while target_i < len(pts) - 1:
            px = pts[target_i].pose.position.x
            py = pts[target_i].pose.position.y
            if math.hypot(px - rx, py - ry) >= lookahead:
                break
            target_i += 1

        tx = pts[target_i].pose.position.x
        ty = pts[target_i].pose.position.y

        # 3) Compute heading error to target
        angle_to_target = math.atan2(ty - ry, tx - rx)
        heading_error = wrap_to_pi(angle_to_target - ryaw)

        # 4) Convert heading error into commands
        k_ang = float(self.get_parameter("k_ang").value)
        max_w = float(self.get_parameter("max_angular").value)
        wz = clamp(k_ang * heading_error, -max_w, +max_w)

        v_max = float(self.get_parameter("linear_speed").value)

        # Simple speed reduction on sharp turns
        # (students can tune / change this easily)
        if abs(heading_error) > 1.0:
            vx = 0.05
        elif abs(heading_error) > 0.6:
            vx = 0.12
        else:
            vx = v_max

        self.publish_cmd(vx, wz)


def clamp(x, lo, hi):
    if x < lo:
        return lo
    if x > hi:
        return hi
    return x


def main():
    rclpy.init()
    node = LocalPathFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Safety stop on exit
        node.publish_cmd(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

