#!/usr/bin/env python3
"""
Bug-style navigation (Bug2) for ROS 2 (rclpy).
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
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


class BugNavigator(Node):
    def __init__(self):
        super().__init__("bug_navigator")

        # Topics / frames
        self.declare_parameter("world_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("goal_topic", "/goal")

        # Behavior parameters
        self.declare_parameter("linear_speed", 0.20)
        self.declare_parameter("angular_speed", 0.6)
        self.declare_parameter("goal_tolerance", 0.05)
        self.declare_parameter("wall_dist", 0.4)
        self.declare_parameter("mline_tol", 0.10)

        self.world_frame = self.get_parameter("world_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.linear_speed = self.get_parameter("linear_speed").value
        self.angular_speed = self.get_parameter("angular_speed").value
        self.goal_tolerance = self.get_parameter("goal_tolerance").value
        self.wall_dist = self.get_parameter("wall_dist").value
        self.mline_tol = self.get_parameter("mline_tol").value

        # Create cmd_vel publisher
        self.cmd_pub = self.create_publisher(
            Twist,
            self.get_parameter("cmd_vel_topic").value,
            10
        )
        # Create scan subscriber
        self.scan_sub = self.create_subscription(
            LaserScan,
            self.get_parameter("scan_topic").value,
            self.on_scan,
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
        self.goal = None             # last goal received - PoseStamped msg
        self.scan = None             # last scan received - LaserScan msg
        self.state = "IDLE"          # IDLE, GO_TO_GOAL, WALL_FOLLOW
        self.start = None            # (x, y) start point
        self.hit_point = None        # (x, y) where we first hit the obstacle
        self.hit_goal_dist = None    # distance to goal at hit point

        # Start control loop timer
        self.timer = self.create_timer(0.05, self.step)  # 20 Hz

        self.get_logger().info("BugNavigator ready.")

    # ----------------------------
    # Get current pose from TF
    # ----------------------------
    def get_pose(self):
        """Return (x, y, yaw), the base_frame pose expressed in world_frame."""
        try:
            tf = self.tf_buffer.lookup_transform(
                target_frame=self.world_frame,
                source_frame=self.base_frame,
                time=rclpy.time.Time(),
                timeout=Duration(seconds=0.2),
            )
        except Exception:
            return None

        x = tf.transform.translation.x
        y = tf.transform.translation.y
        yaw = yaw_from_quat(tf.transform.rotation)
        return (x, y, yaw)

    # ----------------------------
    # Subscriber Callbacks
    # ----------------------------
    def on_goal(self, msg):
        """Goal topic cb: store goal and start point, set GO_TO_GOAL state."""
        # ------------------------------------------------------------
        # TODO: store msg in self.goal, get current pose from TF and 
        #  and save it in self.start, re-init self.hit_point and 
        #  self.hit_goal_dist, set self.state to "GO_TO_GOAL"
        self.goal = msg
        pose = self.get_pose()
        if pose is None:
            self.get_logger().warn("Got goal but no TF pose yet.")
            return
        x, y, _ = pose
        self.start = (x, y)
        self.hit_point = None
        self.hit_goal_dist = None
        self.state = "GO_TO_GOAL"
        self.get_logger().info(
            "Goal received: "
            f"({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})"
        )
        # ------------------------------------------------------------

    def on_scan(self, msg):
        """Scan topic cb: store last LaserScan msg received."""
        # ------------------------------------------------------------
        # TODO: store the LaserScan msg into self.scan
        self.scan = msg
        # ------------------------------------------------------------

    # ----------------------------
    # LaserScan helper functions
    # ----------------------------
    def min_range_in_sector(self, center_deg, half_width_deg):
        """
        Return minimum scan range in an angular sector centered in 'center_deg'
            with a width of twice 'half_width_deg'.

        (Assuming scan.angle_min = 0 and scan.angle_max = 2 * math.pi)
        """
        if self.scan is None:
            return float("inf")

        # ------------------------------------------------------------
        # TODO: Compute start and end of the angular sector
        angle_start = math.radians(center_deg - half_width_deg)
        angle_end = math.radians(center_deg + half_width_deg)
        # TODO: Normalize angles to [0, 2*pi)
        angle_start = angle_start % (2 * math.pi)
        angle_end = angle_end % (2 * math.pi)

        # TODO: Get the start and end indices from the corresponding angles
        i_start = round(angle_start / self.scan.angle_increment)
        i_end = round(angle_end / self.scan.angle_increment)

        # TODO: function to find min in array discarding inf and nan
        def finite_min(seq):
            best = float("inf")
            for r in seq:
                if math.isfinite(r):
                    best = min(best, r)
            return best
        # TODO: find the minimum range value between i_start and i_end
        # handle the case in which i_start > i_end !!!
        if i_start <= i_end:
            # the sector doesn't cross 0 angle
            d = finite_min(self.scan.ranges[i_start:i_end+1])
        else:
            # the sector crosses 0 angle
            dist_1 = finite_min(self.scan.ranges[0:i_end+1])
            dist_2 = finite_min(self.scan.ranges[i_start:-1])
            d = min(dist_1, dist_2)

        return d
        # ------------------------------------------------------------

    def front_dist(self):
        """Return minimum scan distance in the front sector."""
        return self.min_range_in_sector(0.0, 30.0)

    def left_dist(self):
        """Return minimum scan distance in the left sector."""
        return self.min_range_in_sector(72, 10.0)

    # ----------------------------
    # Geometry utils
    # ----------------------------
    def goal_distance(self, x, y):
        """Compute distance of (x, y) from current goal."""
        gx = self.goal.pose.position.x
        gy = self.goal.pose.position.y
        return math.sqrt((gx - x)**2 + (gy - y)**2)
        
    def dist_to_mline(self, x, y):
        """Perpendicular distance from point (x,y) to the m-line."""
        if self.start is None or self.goal is None:
            return float("inf")

        # ------------------------------------------------------------
        # TODO: compute the distance of (x, y) from the start-goal line

        # the m-line connects the start to the goal point
        x_start, y_start = self.start
        x_goal, y_goal = self.goal.pose.position.x, self.goal.pose.position.y

        # Line coefficients: a*x + b*y + c = 0
        a = y_start - y_goal
        b = x_goal - x_start
        c = x_start * y_goal - x_goal * y_start

        denom = math.sqrt(a**2 + b**2)
        if denom < 1e-6:
            # start and goal coincide
            dx = x - x_start
            dy = y - y_start
            return math.sqrt(dx**2 + dy**2)

        # Perpendicular distance from (x, y) to the line
        return abs(a * x + b * y + c) / denom

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
        gx = self.goal.pose.position.x
        gy = self.goal.pose.position.y

        desired = math.atan2(gy - y, gx - x)
        err = angle_wrap(desired - yaw)

        # Simple proportional steering: turn more if angle error is large
        wz = clamp(err * 1.5, -self.angular_speed, self.angular_speed)

        # Slow down if not facing goal
        vx = self.linear_speed * max(0.0, 1.0 - abs(err) / math.radians(70))

        self.publish_cmd(vx, wz)

    def follow_left_wall(self):
        """Publish a twist msg that drives the robot along the left wall."""
        # ------------------------------------------------------------
        # TODO: publish a cmd_vel to follow the wall on the left
        front = self.front_dist()
        left = self.left_dist()

        # Obstacle in front -> turn right
        if front < self.wall_dist:
            vx = 0.0
            wz = -self.angular_speed

        # Wall too far on the left -> turn left
        elif left > 1.25 * self.wall_dist:
            vx = 0.0
            wz = self.angular_speed

        # Otherwise: go forward
        else:
            vx = self.linear_speed
            wz = 0.0

        self.publish_cmd(vx, wz)
        # ------------------------------------------------------------

    # ----------------------------
    # Main loop
    # ----------------------------
    def step(self):
        """Step function called from the control loop timer."""
        if self.state == "IDLE":  # nothing to do
            return

        if self.goal is None or self.scan is None:  # no msg received
            return

        # get current robot pose
        pose = self.get_pose()
        if pose is None:
            return

        x, y, yaw = pose

        # Check if goal reached
        if self.goal_distance(x, y) < self.goal_tolerance:
            self.stop()
            self.state = "IDLE"
            self.get_logger().info("Goal reached.")
            return

        if self.state == "GO_TO_GOAL":
            if self.front_dist() < self.wall_dist:
                # Record hit point
                self.hit_point = (x, y)
                self.hit_goal_dist = self.goal_distance(x, y)
                self.state = "WALL_FOLLOW"
                self.get_logger().info(
                    "Obstacle hit -> switching to WALL_FOLLOW"
                )
                return
            self.go_to_goal(x, y, yaw)
            return

        if self.state == "WALL_FOLLOW":
            # Condition to leave wall in:
            # - close to m-line
            # - closer to goal than at the hit point
            if self.dist_to_mline(x, y) < self.mline_tol:
                if self.goal_distance(x, y) < self.hit_goal_dist * 0.80:
                    # Try to go to goal again if not blocked
                    if self.front_dist() > self.wall_dist:
                        self.state = "GO_TO_GOAL"
                        self.get_logger().info(
                            "Rejoin m-line -> switching to GO_TO_GOAL"
                        )
                        return
            self.follow_left_wall()
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

