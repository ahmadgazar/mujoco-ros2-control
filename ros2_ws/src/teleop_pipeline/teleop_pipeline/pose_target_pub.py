#!/usr/bin/env python3
"""Target pose publisher for teleoperation."""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


class PoseTargetPub(Node):
    """Publishes target end-effector poses."""
    
    def __init__(self):
        super().__init__('pose_target_pub')
        
        self.declare_parameter('position', [0.5, 0.0, 0.4])
        self.declare_parameter('orientation_xyzw', [0.0, 0.0, 0.0, 1.0])
        
        self.pub = self.create_publisher(PoseStamped, '/teleop/pose_target', 10)
        self.create_timer(0.01, self.publish_target)

    def publish_target(self):
        """Publish target pose."""
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
