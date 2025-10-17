#!/usr/bin/env python3
"""Target pose publisher for teleoperation."""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from geometry_msgs.msg import PoseStamped


class PoseTargetPub(Node):
    """Publishes target end-effector poses."""
    
    def __init__(self):
        super().__init__('pose_target_pub')
        
        self.declare_parameter('position', [0.5, 0.0, 0.4])
        self.declare_parameter('orientation_xyzw', [0.0, 0.0, 0.0, 1.0])
        self.declare_parameter('startup_delay', 2.0)  # Delay before publishing (seconds)
        
        # Use volatile QoS to prevent message latching
        qos = QoSProfile(depth=10, durability=QoSDurabilityPolicy.VOLATILE)
        self.pub = self.create_publisher(PoseStamped, '/teleop/pose_target', qos)
        
        # Wait before starting to publish
        delay = self.get_parameter('startup_delay').get_parameter_value().double_value
        self.start_time = self.get_clock().now()
        self.started = False
        
        self.get_logger().info(f"Waiting {delay} seconds before publishing target...")
        self.create_timer(0.01, self.publish_target)

    def publish_target(self):
        """Publish target pose after startup delay."""
        delay = self.get_parameter('startup_delay').get_parameter_value().double_value
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        
        if elapsed < delay:
            # Log countdown every second
            remaining = delay - elapsed
            if not hasattr(self, '_last_countdown') or int(remaining) != self._last_countdown:
                self._last_countdown = int(remaining)
                if self._last_countdown > 0:
                    self.get_logger().info(f"Waiting {self._last_countdown}s before publishing target...")
            return
        
        if not self.started:
            self.started = True
            self.get_logger().info("✅ Now publishing target pose")
        
        p = self.get_parameter('position').get_parameter_value().double_array_value
        o = self.get_parameter('orientation_xyzw').get_parameter_value().double_array_value
        
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = p
        msg.pose.orientation.x, msg.pose.orientation.y = o[0], o[1]
        msg.pose.orientation.z, msg.pose.orientation.w = o[2], o[3]
        
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = PoseTargetPub()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
