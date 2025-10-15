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
        self.pub_cmd.publish(msg)
    
    def forward_state(self, msg: JointState):
        """Forward joint states to controller."""
        self.pub_state.publish(msg)


def main():
    rclpy.init()
    node = TorqueRouter()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
