#!/usr/bin/env python3
"""
TurtleBot4 baseline movement script:
- Rotate in place by +90 degrees (relative yaw)
- Stop
- Wait for a keypress
- Repeat

Publishes: /cmd_vel_unstamped (geometry_msgs/msg/Twist)

Notes:
- This is open-loop using time + approximate angular velocity.
- It's good enough as a foundation for "rotate, scan, rotate" workflows.
- Later replace timing with odometry/IMU-based yaw control for accuracy.
"""

import sys
import termios
import tty
import select
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


def get_key(timeout: Optional[float] = None) -> Optional[str]:
    """
    Non-blocking key read from stdin with optional timeout (seconds).
    Returns the pressed key as a string, or None if nothing was pressed.
    """
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if rlist:
            return sys.stdin.read(1)
        return None
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


class RotateWait(Node):
    def __init__(self):
        super().__init__('tb4_rotate_wait')

        # Publisher to base velocity command
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_unstamped', 10)

        # Tunables (start conservative)
        self.angular_speed = 0.6  # rad/s (positive = CCW)
        self.angle_deg = 90.0     # rotate this many degrees per step


        # Derived timing (open-loop)
        self.angle_rad = self.angle_deg * 3.141592653589793 / 180.0
        self.rotate_time = self.angle_rad / abs(self.angular_speed)

        self.get_logger().info(
            f"Ready. Each step rotates {self.angle_deg:.1f}° "
            f"at {self.angular_speed:.2f} rad/s (≈ {self.rotate_time:.2f}s)."
        )
        self.get_logger().info("Controls: [SPACE]=rotate 90°, [q]=quit")

        # Safety: ensure stop on shutdown
        self._stopped = False

    def publish_twist(self, lin_x: float = 0.0, ang_z: float = 0.0):
        msg = Twist()
        msg.linear.x = float(lin_x)
        msg.angular.z = float(ang_z)
        self.cmd_pub.publish(msg)

    def stop_robot(self):
        # publish a few zeros to ensure the base actually stops
        for _ in range(5):
            self.publish_twist(0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.05)
        self._stopped = True

    def rotate_in_place_90(self):
        self._stopped = False
        start = self.get_clock().now()

        # Command rotation
        self.publish_twist(0.0, self.angular_speed)

        # Hold rotation command for computed duration
        while rclpy.ok():
            elapsed = (self.get_clock().now() - start).nanoseconds * 1e-9
            if elapsed >= self.rotate_time:
                break
            rclpy.spin_once(self, timeout_sec=0.02)

        # Stop
        self.stop_robot()

    def run(self):
        try:
            while rclpy.ok():
                key = get_key(timeout=0.1)
                if key is None:
                    rclpy.spin_once(self, timeout_sec=0.0)
                    continue

                if key.lower() == 'q':
                    self.get_logger().info("Quit requested.")
                    break

                if key == ' ':
                    self.get_logger().info("Rotating +90° ...")
                    self.rotate_in_place_90()
                    self.get_logger().info("Stopped. Waiting for next keypress.")
        finally:
            # Ensure robot stops if node exits
            self.stop_robot()


def main():
    rclpy.init()
    node = RotateWait()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
