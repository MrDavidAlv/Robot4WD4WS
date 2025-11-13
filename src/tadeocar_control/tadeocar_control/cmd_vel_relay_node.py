#!/usr/bin/env python3
"""
Cmd_vel Relay Node for TadeoeCar
Relays Nav2 velocity commands from /cmd_vel to /cmd_vel_nav
This avoids topic conflicts between Xbox control and Nav2
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CmdVelRelayNode(Node):
    """
    Simple relay node that forwards velocity commands from Nav2

    Topic flow:
    - Nav2 (velocity_smoother) → /cmd_vel
    - This relay → /cmd_vel_nav
    - fourws_kinematics subscribes to /cmd_vel_nav for autonomous control

    This separation allows:
    - Xbox control → /cmd_vel_xbox → fourws_kinematics (manual)
    - Nav2 → /cmd_vel → relay → /cmd_vel_nav → fourws_kinematics (autonomous)
    """

    def __init__(self):
        super().__init__('cmd_vel_relay_node')

        self.get_logger().info('Cmd_vel Relay Node starting...')

        # Publisher for relayed commands
        self.cmd_vel_nav_pub = self.create_publisher(
            Twist,
            '/cmd_vel_nav',
            10
        )

        # Subscriber to Nav2 velocity commands
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.get_logger().info('Relay: /cmd_vel → /cmd_vel_nav')
        self.get_logger().info('Cmd_vel Relay Node initialized successfully')

    def cmd_vel_callback(self, msg):
        """
        Relay velocity commands from Nav2 to the kinematics controller

        Args:
            msg: Twist message from Nav2 velocity smoother
        """
        # Simply forward the message to /cmd_vel_nav
        self.cmd_vel_nav_pub.publish(msg)

        # Optional: Log for debugging (commented out to reduce spam)
        # self.get_logger().debug(
        #     f'Relaying: vx={msg.linear.x:.2f}, vy={msg.linear.y:.2f}, wz={msg.angular.z:.2f}'
        # )


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelRelayNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
