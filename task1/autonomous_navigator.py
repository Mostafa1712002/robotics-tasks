#!/usr/bin/env python3
"""
Autonomous TurtleBot3 Navigator
Uses LiDAR scan data to avoid obstacles and navigate through the arena
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped
import numpy as np


class AutonomousNavigator(Node):
    def __init__(self):
        super().__init__('autonomous_navigator')

        # Publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(
            TwistStamped,
            '/cmd_vel',
            10
        )

        # Subscriber for LiDAR scan data
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # Navigation parameters
        self.linear_speed = 0.22  # m/s
        self.angular_speed = 0.5  # rad/s
        self.safe_distance = 0.5  # meters
        self.min_front_distance = float('inf')

        self.get_logger().info('Autonomous Navigator Started!')
        self.get_logger().info('Robot will navigate avoiding obstacles...')

    def scan_callback(self, msg):
        """Process LiDAR scan and decide movement"""

        # Divide scan into regions
        ranges = np.array(msg.ranges)
        ranges[ranges == 0] = float('inf')  # Replace 0 with inf

        # Define regions (assuming 360 degree scan)
        num_readings = len(ranges)
        region_size = num_readings // 5

        regions = {
            'front': min(min(ranges[0:region_size]),
                        min(ranges[-region_size:])),
            'front_left': min(ranges[region_size:2*region_size]),
            'left': min(ranges[2*region_size:3*region_size]),
            'front_right': min(ranges[-2*region_size:-region_size]),
            'right': min(ranges[-3*region_size:-2*region_size])
        }

        self.navigate(regions)

    def navigate(self, regions):
        """Decide movement based on obstacle regions"""

        msg = TwistStamped()
        msg.header.frame_id = 'base_link'

        linear_x = 0.0
        angular_z = 0.0

        state = 'unknown'

        # Decision logic
        if regions['front'] > self.safe_distance and \
           regions['front_left'] > self.safe_distance and \
           regions['front_right'] > self.safe_distance:
            state = 'clear ahead - moving forward'
            linear_x = self.linear_speed
            angular_z = 0.0

        elif regions['front'] < self.safe_distance:
            state = 'obstacle ahead - turning'
            linear_x = 0.0
            # Turn toward the side with more space
            if regions['left'] > regions['right']:
                angular_z = self.angular_speed  # Turn left
            else:
                angular_z = -self.angular_speed  # Turn right

        elif regions['front_left'] < self.safe_distance:
            state = 'obstacle on front-left - turning right'
            linear_x = self.linear_speed * 0.5
            angular_z = -self.angular_speed * 0.5

        elif regions['front_right'] < self.safe_distance:
            state = 'obstacle on front-right - turning left'
            linear_x = self.linear_speed * 0.5
            angular_z = self.angular_speed * 0.5

        else:
            state = 'default - moving forward'
            linear_x = self.linear_speed
            angular_z = 0.0

        # Log state
        self.get_logger().info(
            f'State: {state} | Front: {regions["front"]:.2f}m | '
            f'FL: {regions["front_left"]:.2f}m | FR: {regions["front_right"]:.2f}m'
        )

        # Set velocities
        msg.twist.linear.x = linear_x
        msg.twist.angular.z = angular_z

        # Publish command
        self.cmd_vel_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    navigator = AutonomousNavigator()

    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop the robot
        msg = TwistStamped()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = 0.0
        msg.twist.angular.z = 0.0
        navigator.cmd_vel_pub.publish(msg)

        navigator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
