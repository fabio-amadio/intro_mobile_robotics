#!/usr/bin/env python3
"""
Point-to-point navigation for ROS 2 (rclpy).
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import Twist, PoseStamped
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
    return (a + math.pi) % (2 * math.pi) - math.pi


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


class Point2PointNav(Node):
    def __init__(self):
        super().__init__("_navigator")

        # Topics / frames
        self.declare_parameter("world_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("goal_topic", "/goal")

        # Behavior parameters
        self.declare_parameter("linear_speed", 0.20)
        self.declare_parameter("angular_speed", 0.6)
        self.declare_parameter("goal_tolerance", 0.05)

        self.world_frame = self.get_parameter("world_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.linear_speed = self.get_parameter("linear_speed").value
        self.angular_speed = self.get_parameter("angular_speed").value
        self.goal_tolerance = self.get_parameter("goal_tolerance").value

        # Create cmd_vel publisher
        self.cmd_pub = self.create_publisher(
            Twist,
            self.get_parameter("cmd_vel_topic").value,
            10
        )
        # Create goal subscriber
        self.goal_sub = self.create_subscription(
            PoseStamped,
            self.get_parameter("goal_topic").value,
            self.on_goal,
            10
        )

        # TF
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Init internal variables
        self.goal = None            # last goal received - PoseStamped
        self.state = "IDLE"         # either IDLE or GO_TO_GOAL

        # Start control loop timer
        self.timer = self.create_timer(0.05, self.step)  # 20 Hz

        self.get_logger().info("Point-to-Point Nav ready.")

    # ----------------------------
    # Get current pose from TF
    # ----------------------------
    def get_pose(self):
        """Return (x, y, yaw), the base_frame pose expressed in world_frame."""
        try:
            # ------------------------------------------------------------
            # TODO: get world_frame -> base_frame from the tf_buffer
            tf = self.tf_buffer.lookup_transform(
                target_frame=self.world_frame,
                source_frame=self.base_frame,
                time=rclpy.time.Time(),
                timeout=Duration(seconds=0.2),
            )
            # ------------------------------------------------------------
        except Exception:
            return None

        # ------------------------------------------------------------
        # TODO: return x, y, yaw (TIP: use yaw_from_quat)
        x = tf.transform.translation.x
        y = tf.transform.translation.y
        yaw = yaw_from_quat(tf.transform.rotation)
        return (x, y, yaw)
        # ------------------------------------------------------------

    # ----------------------------
    # Goal callback
    # ----------------------------
    def on_goal(self, msg):
        """Goal topic cb: store goal and set state to GO_TO_GOAL."""
        # ------------------------------------------------------------
        # TODO: store msg in self.goal and set self.state to "GO_TO_GOAL"
        self.goal = msg
        self.state = "GO_TO_GOAL"
        self.get_logger().info(
            "Goal received: "
            f"({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})"
        )
        # ------------------------------------------------------------

    # ----------------------------
    # Geometry utils
    # ----------------------------
    def goal_distance(self, x, y):
        """Compute distance of (x, y) from current goal."""
        # ------------------------------------------------------------
        # TODO: compute the distance from (x, y) to self.goal
        gx = self.goal.pose.position.x
        gy = self.goal.pose.position.y
        return math.sqrt((gx - x)**2 + (gy - y)**2)
        # ------------------------------------------------------------

    # ----------------------------
    # Motion primitives
    # ----------------------------
    def publish_cmd(self, vx, wz):
        """Publish a twist msg with linear.x = vx and angular.z = wz."""
        msg = Twist()
        msg.linear.x = float(vx)
        msg.angular.z = float(wz)
        self.cmd_pub.publish(msg)

    def stop(self):
        """Publish a zero velocity Twist msg."""
        self.publish_cmd(0.0, 0.0)

    def go_to_goal(self, x, y, yaw):
        """Publish a twist msg that drives the robot towards the goal."""
        # ------------------------------------------------------------
        # TODO: publish a cmd that_vel to go to the goal
        gx = self.goal.pose.position.x
        gy = self.goal.pose.position.y

        desired = math.atan2(gy - y, gx - x)
        err = angle_wrap(desired - yaw)

        # Simple proportional steering: turn more if angle error is large
        wz = clamp(err * 1.5, -self.angular_speed, self.angular_speed)

        # Slow down if not facing goal
        vx = self.linear_speed * max(0.0, 1.0 - abs(err) / math.radians(70))

        self.publish_cmd(vx, wz)
        # ------------------------------------------------------------

    # ----------------------------
    # Main loop
    # ----------------------------
    def step(self):
        """Step function called from the control loop timer."""
        if self.state == "IDLE":  # nothing to do
            return

        if self.goal is None:  # no active goal
            return

        # get current robot pose
        pose = self.get_pose()
        if pose is None:
            return

        x, y, yaw = pose
    
        # ------------------------------------------------------------
        # TODO: stop if goal reached, or continue going to the goal
        if self.goal_distance(x, y) < self.goal_tolerance:
            self.stop()
            self.state = "IDLE"
            self.get_logger().info("Goal reached.")
        else:
            self.go_to_goal(x, y, yaw)
        # ------------------------------------------------------------


def main():
    rclpy.init()
    node = Point2PointNav()
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

