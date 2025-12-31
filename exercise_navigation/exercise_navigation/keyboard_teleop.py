#!/usr/bin/env python3
"""
ROS 2 keyboard teleop node publishing Twist msg.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from pynput import keyboard


class SimpleKeyboardTeleop(Node):
    def __init__(self):
        super().__init__("simple_keyboard_teleop")

        # Parameters (keep it simple)
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("linear_speed", 0.25)
        self.declare_parameter("angular_speed", 1.2)

        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.linear_speed = float(self.get_parameter("linear_speed").value)
        self.angular_speed = float(self.get_parameter("angular_speed").value)

        self.pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        self.pressed = set()  # to track pressed keys

        # ------------------------------------------------------------
        # TODO: define the set of allowed keys between:
        # - {"w", "a", "s", "d"} for QWERTY keyboards
        # - {"z", "q", "s", "d"} for AZERTY keyboards
        self.allowed = {"w", "a", "s", "d"}  # or {"z", "q", "s", "d"}
        # ------------------------------------------------------------

        self.get_logger().info("Simple keyboard teleop started.")
        self.get_logger().info(f"Publishing to: {self.cmd_vel_topic}")

    def publish_cmd_vel(self):
        """
        Compute cmd_vel from pressed keys and publish Twist msg
        """
        # ------------------------------------------------------------
        # TODO: compute and publish the proper velocity command
        # - check self.pressed and decide linear and rotational speed
        # - create and fill-in a Twist msg
        # - publish the Twist msg with self.pub

        vx = 0.0
        wz = 0.0

        if ("w" in self.pressed) and ("s" not in self.pressed):
            vx = +self.linear_speed
        elif ("s" in self.pressed) and ("w" not in self.pressed):
            vx = -self.linear_speed
        if ("a" in self.pressed) and ("d" not in self.pressed):
            wz = +self.angular_speed
        elif ("d" in self.pressed) and ("a" not in self.pressed):
            wz = -self.angular_speed

        msg = Twist()
        msg.linear.x = vx
        msg.angular.z = wz
        self.pub.publish(msg)
        # ------------------------------------------------------------

    def key_to_char(self, key):
        """
        Convert pynput key event to a normalized char (if possible).
        Returns None if key has no char.
        """
        if isinstance(key, keyboard.KeyCode) and key.char:
            ch = key.char.lower()
            return ch
        return None

    def on_press(self, key):
        # ------------------------------------------------------------
        # TODO: handle pressed key
        # - Convert key to lowercase char
        # - If char belongs to self.allowed add it to self.pressed
        # - Otherwise return
        ch = self.key_to_char(key)
        if ch not in self.allowed:
            return

        self.pressed.add(ch)
        # ------------------------------------------------------------

        self.publish_cmd_vel()

    def on_release(self, key):
        # ------------------------------------------------------------
        # TODO: handle released key
        # - Convert key to lowercase char
        # - If char belongs to self.allowed add it to self.pressed
        # - Otherwise return
        ch = self.key_to_char(key)
        if ch not in self.allowed:
            return

        self.pressed.discard(ch)
        # ------------------------------------------------------------

        self.publish_cmd_vel()


def main():
    rclpy.init()
    node = SimpleKeyboardTeleop()

    # A keyboard listener is a threading.Thread
    listener = keyboard.Listener(
        on_press=node.on_press,
        on_release=node.on_release,
    )
    listener.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Safety stop on shutdown
        try:
            node.pub.publish(Twist())
        except Exception:
            pass

        try:
            listener.stop()
        except Exception:
            pass

        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
