#!/usr/bin/env python3
"""MuJoCo simulation bridge for ROS 2."""

import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import JointState
import mujoco
import mujoco.viewer
import threading


class MuJoCoSimBridge(Node):
    """Runs MuJoCo physics simulation and publishes joint states."""
    
    def __init__(self):
        super().__init__('mujoco_sim_bridge')
        
        self.declare_parameter('mjcf_path', '')
        self.declare_parameter('use_robot_descriptions', True)
        self.declare_parameter('robot_description_pkg', 'panda_mj_description')
        self.declare_parameter('sim_dt', 0.002)
        self.declare_parameter('use_viewer', False)
        
        mjcf_path = self.get_parameter('mjcf_path').value
        if self.get_parameter('use_robot_descriptions').value and not mjcf_path:
            try:
                from robot_descriptions import panda_mj_description
                mjcf_path = panda_mj_description.MJCF_PATH
                self.get_logger().info(f"Loaded model: {mjcf_path}")
            except Exception as e:
                self.get_logger().error(f"Failed to load robot_descriptions: {e}")
                return
        
        self.m = mujoco.MjModel.from_xml_path(mjcf_path)
        self.d = mujoco.MjData(self.m)
        
        self.joint_names = [
            self.m.joint(i).name for i in range(self.m.njnt)
            if self.m.jnt_type[i] != mujoco.mjtJoint.mjJNT_FREE
        ]
        self.get_logger().info(f"Simulating joints: {self.joint_names}")
        
        self.pub_state = self.create_publisher(JointState, '/mujoco/joint_states', 10)
        self.sub_cmd = self.create_subscription(JointState, '/mujoco/effort_command', self.on_effort_cmd, 10)
        
        dt = float(self.get_parameter('sim_dt').value)
        self.m.opt.timestep = dt
        self.create_timer(dt, self.sim_step)
        
        if self.get_parameter('use_viewer').value:
            self.get_logger().info("Starting MuJoCo viewer...")
            viewer_thread = threading.Thread(
                target=lambda: mujoco.viewer.launch(self.m, self.d),
                daemon=True
            )
            viewer_thread.start()
        
        self.effort_cmd = np.zeros(self.m.nu)
        
    def on_effort_cmd(self, msg: JointState):
        """Receive torque commands."""
        if msg.effort:
            n = min(len(msg.effort), self.m.nu)
            self.effort_cmd[:n] = msg.effort[:n]
    
    def sim_step(self):
        """Step simulation and publish state."""
        self.d.ctrl[:] = self.effort_cmd
        mujoco.mj_step(self.m, self.d)
        
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = self.joint_names
        js.position = self.d.qpos[:len(self.joint_names)].tolist()
        js.velocity = self.d.qvel[:len(self.joint_names)].tolist()
        js.effort = self.d.qfrc_actuator[:len(self.joint_names)].tolist()
        self.pub_state.publish(js)

def main():
    rclpy.init()
    node = MuJoCoSimBridge()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

