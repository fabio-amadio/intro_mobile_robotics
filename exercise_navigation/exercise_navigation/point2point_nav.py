#!/usr/bin/env python3
"""
Point-to-point navigation for ROS 2 (rclpy).
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import Twist, PoseStamped
from builtin_interfaces.msg import Time
import tf2_ros


def angle_wrap(a):
    """
    Normalize an angle to the interval [-pi, pi].

    Parameters
    ----------
    a : float
        Angle in radians (can be any real value).

    Returns
    -------
    float
        Equivalent angle in the range [-pi, pi].
    """
    while a > math.pi:
        a = a - 2.0 * math.pi
    while a < -math.pi:
        a = a + 2.0 * math.pi
    return a


def yaw_from_quat(q):
    """
    Extract the yaw (rotation around the Z axis) from a quaternion.

    Parameters
    ----------
    q : geometry_msgs.msg.Quaternion
        Quaternion representing 3D orientation.

    Returns
    -------
    float
        Yaw angle in radians, in the range [-pi, pi].
    """
    x, y, z, w = q.x, q.y, q.z, q.w
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def clamp(x, lo, hi):
    """
    Clamp a value between a lower and an upper bound.

    Parameters
    ----------
    x : float
        Value to clamp.
    lo : float
        Lower bound.
    hi : float
        Upper bound.

    Returns
    -------
    float
        Clamped value.
    """
    return lo if x < lo else hi if x > hi else x


class BugNavigator(Node):
    def __init__(self):
        super().__init__("bug_navigator")

        # Topics / frames
        self.declare_parameter("world_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("goal_topic", "/goal")

        # Behavior parameters
        self.declare_parameter("linear_speed", 0.20)
        self.declare_parameter("angular_speed", 0.6)
        self.declare_parameter("goal_tolerance", 0.05)

        self.linear_speed = self.get_parameter("linear_speed").value
        self.angular_speed = self.get_parameter("angular_speed").value
        self.goal_tolerance = self.get_parameter("goal_tolerance").value
        self.world_frame = self.get_parameter("world_frame").value
        self.base_frame = self.get_parameter("base_frame").value

        # Create topic publishers and subscribers
        self.cmd_pub = self.create_publisher(Twist, self.get_parameter("cmd_vel_topic").value, 10)
        self.goal_sub = self.create_subscription(PoseStamped, self.get_parameter("goal_topic").value, self.on_goal, 10)

        # TF
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # State
        self.goal = None

        self.state = "IDLE"          # IDLE, GO_TO_GOAL, DONE
        self.start = None            # (x, y) start point

        # Control loop
        self.timer = self.create_timer(0.05, self.step)  # 20 Hz

        self.get_logger().info("Point-to-Point Nav ready.")

    # ----------------------------
    # ROS callback
    # ----------------------------
    def on_goal(self, msg):
        self.goal = msg
        pose = self.get_pose()
        if pose is None:
            self.get_logger().warn("Got goal but no TF pose yet.")
            return
        x, y, _ = pose
        self.start = (x, y)
        self.state = "GO_TO_GOAL"
        self.get_logger().info(f"Goal received: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})")

    # ----------------------------
    # TF pose
    # ----------------------------
    def get_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                target_frame=self.world_frame,
                source_frame=self.base_frame,
                time=Time(),
                timeout=Duration(seconds=0.2),
            )
        except Exception:
            return None

        x = tf.transform.translation.x
        y = tf.transform.translation.y
        yaw = yaw_from_quat(tf.transform.rotation)
        return (x, y, yaw)

    # ----------------------------
    # Geometry
    # ----------------------------

    def goal_distance(self, x, y):
        gx = self.goal.pose.position.x
        gy = self.goal.pose.position.y
        return math.hypot(gx - x, gy - y)

    # ----------------------------
    # Motion primitives
    # ----------------------------
    def publish_cmd(self, vx, wz):
        msg = Twist()
        msg.linear.x = float(vx)
        msg.angular.z = float(wz)
        self.cmd_pub.publish(msg)

    def stop(self):
        self.publish_cmd(0.0, 0.0)

    def go_to_goal_cmd(self, x, y, yaw):
        gx = self.goal.pose.position.x
        gy = self.goal.pose.position.y

        desired = math.atan2(gy - y, gx - x)
        err = angle_wrap(desired - yaw)

        # Simple proportional steering: turn more if angle error is large
        wz = clamp(err * 1.5, -self.angular_speed, self.angular_speed)

        # Slow down if not facing goal
        vx = self.linear_speed * max(0.0, 1.0 - abs(err) / math.radians(70))

        return vx, wz

    # ----------------------------
    # Main loop
    # ----------------------------
    def step(self):
        if self.state in ["IDLE", "DONE"]:
            return

        if self.goal is None:
            return

        pose = self.get_pose()
        if pose is None:
            return

        x, y, yaw = pose

        # Check if goal reached
        if self.goal_distance(x, y) < self.goal_tolerance:
            self.stop()
            self.state = "DONE"
            self.get_logger().info("Goal reached.")
            return

        if self.state == "GO_TO_GOAL":
            vx, wz = self.go_to_goal_cmd(x, y, yaw)
            self.publish_cmd(vx, wz)
            return


def main():
    rclpy.init()
    node = BugNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

