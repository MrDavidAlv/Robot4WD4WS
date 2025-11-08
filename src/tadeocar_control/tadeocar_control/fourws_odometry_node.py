#!/usr/bin/env python3
"""
4WD4WS Odometry Node for TadeoeCar
Calculates accurate odometry using all 4 wheels and their steering angles
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster
import math
import numpy as np


class FourWSOdometryNode(Node):
    """
    4WD4WS Odometry Calculator

    Calculates robot odometry by reading:
    - Wheel velocities from joint_states (4 wheels)
    - Steering angles from joint_states (4 steering joints)

    Uses forward kinematics to compute robot velocity and integrates
    to get position (x, y, theta)
    """

    def __init__(self):
        super().__init__('fourws_odometry_node')

        # Robot parameters (from TadeoeCar model)
        self.wheel_radius = 0.1  # meters
        self.wheel_base = 1.058  # meters (front to rear axle)
        self.track_width = 0.55  # meters (left to right wheel)

        # Odometry state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = None

        # Current joint states
        self.wheel_velocities = {
            'front_left': 0.0,
            'front_right': 0.0,
            'rear_left': 0.0,
            'rear_right': 0.0
        }

        self.steering_angles = {
            'front_left': 0.0,
            'front_right': 0.0,
            'rear_left': 0.0,
            'rear_right': 0.0
        }

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/odom', 50)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscriber
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Timer for odometry calculation (50Hz)
        self.create_timer(0.02, self.update_odometry)

        self.get_logger().info('4WD4WS Odometry Node initialized')
        self.get_logger().info(f'Wheel radius: {self.wheel_radius}m')
        self.get_logger().info(f'Wheel base: {self.wheel_base}m')
        self.get_logger().info(f'Track width: {self.track_width}m')

    def joint_state_callback(self, msg):
        """Update wheel velocities and steering angles from joint states"""
        try:
            for i, name in enumerate(msg.name):
                # Update wheel velocities
                if 'wheel_joint' in name:
                    if 'front_left' in name:
                        self.wheel_velocities['front_left'] = msg.velocity[i]
                    elif 'front_right' in name:
                        self.wheel_velocities['front_right'] = msg.velocity[i]
                    elif 'rear_left' in name:
                        self.wheel_velocities['rear_left'] = msg.velocity[i]
                    elif 'rear_right' in name:
                        self.wheel_velocities['rear_right'] = msg.velocity[i]

                # Update steering angles
                elif 'steering_joint' in name:
                    if 'front_left' in name:
                        self.steering_angles['front_left'] = msg.position[i]
                    elif 'front_right' in name:
                        self.steering_angles['front_right'] = msg.position[i]
                    elif 'rear_left' in name:
                        self.steering_angles['rear_left'] = msg.position[i]
                    elif 'rear_right' in name:
                        self.steering_angles['rear_right'] = msg.position[i]
        except Exception as e:
            self.get_logger().warn(f'Error reading joint states: {e}')

    def update_odometry(self):
        """Calculate and publish odometry"""
        current_time = self.get_clock().now()

        if self.last_time is None:
            self.last_time = current_time
            return

        # Calculate dt
        dt = (current_time - self.last_time).nanoseconds / 1e9
        if dt <= 0.0:
            return

        # Calculate robot velocity using 4WS kinematics
        vx, vy, wz = self.calculate_robot_velocity()

        # Integrate to get position
        # Using Runge-Kutta 2nd order for better accuracy
        delta_x = (vx * math.cos(self.theta) - vy * math.sin(self.theta)) * dt
        delta_y = (vx * math.sin(self.theta) + vy * math.cos(self.theta)) * dt
        delta_theta = wz * dt

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta

        # Normalize theta to [-pi, pi]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # Publish odometry message
        self.publish_odometry(current_time, vx, vy, wz)

        # Publish TF transform
        self.publish_transform(current_time)

        self.last_time = current_time

    def calculate_robot_velocity(self):
        """
        Calculate robot velocity (vx, vy, wz) from wheel velocities and steering angles

        For 4WD4WS with individual steering, we use the instantaneous center of rotation (ICR)
        method or average the velocity contributions from each wheel.

        Returns:
            tuple: (vx, vy, wz) - linear x, linear y, angular z velocities in robot frame
        """
        # Convert wheel angular velocities to linear velocities
        wheel_lin_vel = {
            key: vel * self.wheel_radius
            for key, vel in self.wheel_velocities.items()
        }

        # Calculate velocity contribution from each wheel
        # Each wheel contributes to robot velocity based on its steering angle
        velocities = []

        for wheel_name in ['front_left', 'front_right', 'rear_left', 'rear_right']:
            v_wheel = wheel_lin_vel[wheel_name]
            angle = self.steering_angles[wheel_name]

            # Wheel velocity in robot frame
            vx_wheel = v_wheel * math.cos(angle)
            vy_wheel = v_wheel * math.sin(angle)

            velocities.append((vx_wheel, vy_wheel))

        # Average the velocities from all wheels
        # This works well for omnidirectional and crab modes
        vx = sum(v[0] for v in velocities) / 4.0
        vy = sum(v[1] for v in velocities) / 4.0

        # Calculate angular velocity using wheel positions and velocities
        # Use the difference between left and right wheel velocities
        # and front and rear wheel orientations

        # For rotation: wheels should have velocity perpendicular to radius from center
        # Simplified: use average of front and rear axle angular contributions

        # Front axle contribution
        fl_angle = self.steering_angles['front_left']
        fr_angle = self.steering_angles['front_right']
        fl_vel = wheel_lin_vel['front_left']
        fr_vel = wheel_lin_vel['front_right']

        # Rear axle contribution
        rl_angle = self.steering_angles['rear_left']
        rr_angle = self.steering_angles['rear_right']
        rl_vel = wheel_lin_vel['rear_left']
        rr_vel = wheel_lin_vel['rear_right']

        # Calculate angular velocity from wheel velocities
        # For omnidirectional mode, all wheels contribute to rotation
        # Approximate using velocity difference and geometry

        # Simplified approach: detect rotation from steering angle patterns
        # If wheels are in "diamond" pattern (±45°), it's pure rotation
        avg_angle = (abs(fl_angle) + abs(fr_angle) + abs(rl_angle) + abs(rr_angle)) / 4.0

        if avg_angle > 0.6:  # ~35 degrees, likely rotation mode
            # Pure or combined rotation
            # Use wheel distance from center
            wheel_distance = math.sqrt((self.wheel_base/2)**2 + (self.track_width/2)**2)
            avg_wheel_vel = (abs(fl_vel) + abs(fr_vel) + abs(rl_vel) + abs(rr_vel)) / 4.0
            wz = avg_wheel_vel / wheel_distance

            # Determine rotation direction from wheel velocities
            if fl_vel < 0:  # Assuming positive rotation has positive velocities
                wz = -wz
        else:
            # Linear motion or ackermann - minimal rotation
            # Calculate from velocity difference
            v_left = (fl_vel + rl_vel) / 2.0
            v_right = (fr_vel + rr_vel) / 2.0
            wz = (v_right - v_left) / self.track_width

        return vx, vy, wz

    def publish_odometry(self, current_time, vx, vy, wz):
        """Publish odometry message"""
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        # Position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        # Orientation (quaternion from theta)
        odom.pose.pose.orientation = self.quaternion_from_euler(0, 0, self.theta)

        # Velocity
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz

        # Covariance (tuned for 4WD4WS with individual steering)
        # Higher covariance due to slip and approximations in the model
        odom.pose.covariance[0] = 0.05   # x
        odom.pose.covariance[7] = 0.05   # y
        odom.pose.covariance[14] = 0.0   # z (not used)
        odom.pose.covariance[21] = 0.0   # roll (not used)
        odom.pose.covariance[28] = 0.0   # pitch (not used)
        odom.pose.covariance[35] = 0.08  # yaw (higher uncertainty in rotation)

        odom.twist.covariance[0] = 0.05
        odom.twist.covariance[7] = 0.05
        odom.twist.covariance[35] = 0.08

        self.odom_pub.publish(odom)

    def publish_transform(self, current_time):
        """Publish TF transform from odom to base_link"""
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        # Translation
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0

        # Rotation
        t.transform.rotation = self.quaternion_from_euler(0, 0, self.theta)

        self.tf_broadcaster.sendTransform(t)

    @staticmethod
    def quaternion_from_euler(roll, pitch, yaw):
        """Convert Euler angles to quaternion"""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = Quaternion()
        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy

        return q


def main(args=None):
    rclpy.init(args=args)
    node = FourWSOdometryNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
