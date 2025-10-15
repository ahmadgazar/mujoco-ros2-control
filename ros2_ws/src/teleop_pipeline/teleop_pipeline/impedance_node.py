#!/usr/bin/env python3
"""Joint-space impedance controller for ROS 2."""

import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import JointState


class ImpedanceController(Node):
    """Computes control torques: τ = K(q_d - q) + D(q̇_d - q̇)"""
    
    def __init__(self):
        super().__init__('impedance')
        
        self.declare_parameter('joint_names', [''])
        self.declare_parameter('k_diag', [80.0, 80.0, 60.0, 40.0, 30.0, 20.0])
        self.declare_parameter('d_diag', [4.0, 4.0, 3.0, 2.0, 2.0, 1.0])
        
        self.joint_names = list(self.get_parameter('joint_names').value)
        self.K = np.diag(self.get_parameter('k_diag').value)
        self.D = np.diag(self.get_parameter('d_diag').value)
        
        self.q = None
        self.dq = None
        self.qd = None
        self.dqd = None
        
        self.create_subscription(JointState, '/ctrl/q_ref', self.on_reference, 50)
        self.create_subscription(JointState, '/robot/state', self.on_state, 200)
        self.pub_tau = self.create_publisher(JointState, '/ctrl/torques', 200)
        self.create_timer(0.002, self.control_loop)

    def on_reference(self, msg: JointState):
        """Receive desired joint states."""
        self.qd = np.array(msg.position)
        self.dqd = np.array(msg.velocity) if msg.velocity else np.zeros_like(self.qd)
    
    def on_state(self, msg: JointState):
        """Receive current joint states."""
        self.q = np.array(msg.position)
        self.dq = np.array(msg.velocity) if msg.velocity else np.zeros_like(self.q)
    
    def control_loop(self):
        """Compute and publish control torques."""
        if self.q is None or self.qd is None:
            return
        
        n = len(self.joint_names) if self.joint_names[0] else len(self.q)
        e = self.qd[:n] - self.q[:n]
        ed = self.dqd[:n] - self.dq[:n]
        tau = (self.K[:n, :n] @ e) + (self.D[:n, :n] @ ed)
        
        out = JointState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.name = self.joint_names if self.joint_names[0] else []
        out.effort = tau.tolist()
        self.pub_tau.publish(out)


def main():
    rclpy.init()
    node = ImpedanceController()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
