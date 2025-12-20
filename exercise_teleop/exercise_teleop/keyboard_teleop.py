#!/usr/bin/env python3
"""
Very simple ROS 2 keyboard teleop (WASD) with multi-key support.

Controls (hold keys to combine):
  W = forward
  S = backward
  A = turn left
  D = turn right
  ESC = quit

Behavior:
- Publishes /cmd_vel ONLY when something changes due to a key press/release.
- If you release all keys, it publishes one STOP (0,0).

Why pynput?
- It can detect key press AND key release, so W+A together works properly.
- Terminal input (like input() or curses) is not reliable for real multi-key holds.

Install:
  pip3 install pynput

Run:
  ros2 run <your_pkg> keyboard_teleop_wasd

Params (optional):
  - cmd_vel_topic   (default /cmd_vel)
  - linear_speed    (default 0.25 m/s)
  - angular_speed   (default 1.2 rad/s)
"""

import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from pynput import keyboard


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__("keyboard_teleop_wasd")

        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("linear_speed", 0.25)
        self.declare_parameter("angular_speed", 1.2)

        self.topic = self.get_parameter("cmd_vel_topic").value
        self.linear_speed = float(self.get_parameter("linear_speed").value)
        self.angular_speed = float(self.get_parameter("angular_speed").value)

        self.pub = self.create_publisher(Twist, self.topic, 10)

        # Which keys are currently held down?
        # We'll store characters: 'w', 'a', 's', 'd'
        self.pressed = set()

        # Lock = small safety mechanism because pynput runs in another thread.
        self.lock = threading.Lock()

        # Remember last published command so we only publish when it changes.
        self.last_vx = None
        self.last_wz = None

        self.get_logger().info("Keyboard teleop ready!")
        self.get_logger().info("Hold keys: W/S forward/back, A/D turn left/right.")

    def publish_if_changed(self):
        vx, wz = self.compute_command()

        # Only publish if something changed
        if self.last_vx == vx and self.last_wz == wz:
            return

        msg = Twist()
        msg.linear.x = vx
        msg.angular.z = wz
        self.pub.publish(msg)

        self.last_vx = vx
        self.last_wz = wz

    def compute_command(self):
        # Copy the set safely
        with self.lock:
            p = set(self.pressed)

        # Forward/back (W/S). If both held, they cancel.
        vx = 0.0
        if "w" in p and "s" not in p:
            vx = +self.linear_speed
        elif "s" in p and "w" not in p:
            vx = -self.linear_speed

        # Turn left/right (A/D). If both held, they cancel.
        wz = 0.0
        if "a" in p and "d" not in p:
            wz = +self.angular_speed
        elif "d" in p and "a" not in p:
            wz = -self.angular_speed

        return vx, wz

    def normalize_key(self, key):
        # Convert a key event into 'w','a','s','d' or return None.
        if isinstance(key, keyboard.KeyCode) and key.char is not None:
            ch = key.char.lower()
            if ch in ["w", "a", "s", "d"]:
                return ch
        return None

    def on_press(self, key):
        ch = self.normalize_key(key)
        if ch is None:
            return

        with self.lock:
            self.pressed.add(ch)

        self.publish_if_changed()

    def on_release(self, key):
        ch = self.normalize_key(key)
        if ch is None:
            return

        with self.lock:
            if ch in self.pressed:
                self.pressed.remove(ch)

        self.publish_if_changed()


def main():
    rclpy.init()
    node = KeyboardTeleop()

    listener = keyboard.Listener(
        on_press=node.on_press, 
        on_release=node.on_release
    )
    listener.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Safety stop on shutdown (optional)
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
