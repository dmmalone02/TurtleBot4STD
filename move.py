#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


def move():
    rclpy.init()
    node = Node('move_node')
    publisher = node.create_publisher(Twist, 'cmd_vel_unstamped', 10)

    move_cmd = Twist()
    move_cmd.linear.x = 0.5  # Move forward at 0.5 m/s
    move_cmd.angular.z = 0.0  # No rotation

    publisher.publish(move_cmd)
    rclpy.spin_once(node, timeout_sec=1)  

    node.destroy_node()
    rclpy.shutdown()