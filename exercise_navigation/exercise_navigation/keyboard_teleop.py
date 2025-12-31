#!/usr/bin/env python3
"""
ROS 2 keyboard teleop node publishing Twist msg.
"""

import threading

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

        # TODO(Students):
        # Define the set of allowed keys between:
        # - ("w", "a", "s", "d") for QWERTY keyboards
        # - ("z", "q", "s", "d") for AZERTY keyboards

        # --- SOLUTION ---
        self._allowed_keys = ("w", "a", "s", "d")

        # State: which keys are currently held down
        self._pressed = set()

        # pynput callbacks run in a different thread
        self._lock = threading.Lock()

        self.get_logger().info("Simple keyboard teleop started.")
        self.get_logger().info(f"Publishing to: {self.cmd_vel_topic}")

    def _key_to_char(self, key):
        """
        Convert pynput key event to a normalized char in {"w","a","s","d"}.
        Return None for other keys.
        """
        # TODO(Students):
        # 1) If key is a keyboard.KeyCode and has a .char, normalize to lowercase.
        # 2) If it's one of the self._allowed_keys, return it.
        # 3) Otherwise return None.
        #
        # Hint:
        #   isinstance(key, keyboard.KeyCode)
        #   key.char might be None

        # --- SOLUTION ---
        if isinstance(key, keyboard.KeyCode) and key.char:
            ch = key.char.lower()
            if ch in self._allowed_keys:
                return ch
        return None

    def _compute_cmd(self):
        """
        Compute (vx, wz) based on which keys are currently pressed.

        Rules:
        - W forward, S backward. If both pressed -> 0.
        - A left, D right. If both pressed -> 0.
        """
        with self._lock:
            keys = self._pressed

        # TODO(Students):
        # Implement the logic to compute vx and wz (floats).
        # Use self.linear_speed and self.angular_speed.

        vx = 0.0
        wz = 0.0

        # --- SOLUTION ---
        if ("w" in keys) and ("s" not in keys):
            vx = +self.linear_speed
        elif ("s" in keys) and ("w" not in keys):
            vx = -self.linear_speed
        if ("a" in keys) and ("d" not in keys):
            wz = +self.angular_speed
        elif ("d" in keys) and ("a" not in keys):
            wz = -self.angular_speed

        return vx, wz

    def _publish_cmd_vel(self):
        """
        Publish Twist msg.
        """
        # TODO(Students):
        # - Compute the desired cmd_vel
        # - Create a Twist msg, set linear.x and angular.z, then publish.

        # --- SOLUTION ---
        vx, wz = self._compute_cmd()
        msg = Twist()
        msg.linear.x = vx
        msg.angular.z = wz
        self.pub.publish(msg)

    # ---------- Key event handlers ----------

    def on_press(self, key):
        ch = self._key_to_char(key)
        if ch is None:
            return

        with self._lock:
            self._pressed.add(ch)

        self._publish_cmd_vel()

    def on_release(self, key):
        ch = self._key_to_char(key)
        if ch is None:
            return

        with self._lock:
            self._pressed.discard(ch)

        self._publish_cmd_vel()


def main():
    rclpy.init()
    node = SimpleKeyboardTeleop()

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
