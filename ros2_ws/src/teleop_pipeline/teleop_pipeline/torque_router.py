#!/usr/bin/env python3
"""Message router for teleoperation pipeline."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class TorqueRouter(Node):
    """Routes torques to simulator and joint states to controller."""
    
    def __init__(self):
        super().__init__('torque_router')
        
        self.pub_cmd = self.create_publisher(JointState, '/mujoco/effort_command', 10)
        self.pub_state = self.create_publisher(JointState, '/robot/state', 10)
        
        self.create_subscription(JointState, '/ctrl/torques', self.forward_torques, 50)
        self.create_subscription(JointState, '/mujoco/joint_states', self.forward_state, 200)
    
    def forward_torques(self, msg: JointState):
        """Forward torques to simulator."""
        try:
            # Create a clean message to avoid serialization issues
            clean_msg = JointState()
            clean_msg.header.stamp = msg.header.stamp
            clean_msg.header.frame_id = str(msg.header.frame_id) if msg.header.frame_id else ''
            
            # Ensure names are proper strings
            clean_msg.name = [str(name) for name in msg.name if name]
            
            # Copy data arrays safely
            clean_msg.position = list(msg.position) if msg.position else []
            clean_msg.velocity = list(msg.velocity) if msg.velocity else []
            clean_msg.effort = list(msg.effort) if msg.effort else []
            
            self.pub_cmd.publish(clean_msg)
        except Exception as e:
            self.get_logger().error(f"Error forwarding torques: {e}")
    
    def forward_state(self, msg: JointState):
        """Forward joint states to controller."""
        try:
            # Create a completely fresh message to break DDS cache
            clean_msg = JointState()
            clean_msg.header.stamp = self.get_clock().now().to_msg()  # Use router's own timestamp
            clean_msg.header.frame_id = ''  # Always empty string for proper serialization
            
            # Ensure names are proper strings and synchronized with data length
            names = [str(name) for name in msg.name if name]
            pos_len = len(msg.position) if msg.position else 0
            vel_len = len(msg.velocity) if msg.velocity else 0
            eff_len = len(msg.effort) if msg.effort else 0
            
            # Use minimum length to ensure arrays match
            data_len = min(len(names), pos_len, vel_len, eff_len) if names else 0
            
            if data_len > 0:
                clean_msg.name = names[:data_len]
                clean_msg.position = list(msg.position[:data_len])
                clean_msg.velocity = list(msg.velocity[:data_len])
                clean_msg.effort = list(msg.effort[:data_len])
            
                self.pub_state.publish(clean_msg)
        except Exception as e:
            self.get_logger().error(f"Error forwarding state: {e}")


def main():
    rclpy.init()
    node = TorqueRouter()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
