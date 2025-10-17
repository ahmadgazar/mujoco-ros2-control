#!/usr/bin/env python3
"""
Comprehensive real-time monitor for the mujoco-ros2-control pipeline.
Shows joint states, control commands, end-effector tracking, and target status.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
import numpy as np
import mujoco
from scipy.spatial.transform import Rotation
import time
import os


class PipelineMonitor(Node):
    """Unified monitor for the entire control pipeline."""
    
    def __init__(self):
        super().__init__('pipeline_monitor')
        
        # Load model for forward kinematics
        model_path = "/home/ahmad.gazar/devel/mujoco-ros2-control/models/franka_panda/mjx_scene.xml"
        self.m = mujoco.MjModel.from_xml_path(model_path)
        self.d = mujoco.MjData(self.m)
        self.ee_site_id = self.m.site("attachment_site").id
        
        # State variables
        self.joint_states = None
        self.control_cmd = None  # Position or torque command
        self.pose_target = None
        self.current_q = None
        
        self.target_pos = None
        self.target_rot_mat = None
        self.has_target = False
        
        self.last_joint_time = None
        self.last_control_time = None
        
        # Subscribers
        self.create_subscription(JointState, '/mujoco/joint_states', self.on_joint_states, 10)
        self.create_subscription(JointState, '/robot/state', self.on_robot_state, 10)
        self.create_subscription(JointState, '/ctrl/q_ref', self.on_position_cmd, 10)
        self.create_subscription(JointState, '/ctrl/torques', self.on_torque_cmd, 10)
        self.create_subscription(PoseStamped, '/teleop/pose_target', self.on_pose_target, 10)
        
        # Display timer
        self.create_timer(0.2, self.display_status)  # 5 Hz display
        
        self.get_logger().info("Pipeline monitor started. Press Ctrl+C to exit.")
    
    def on_joint_states(self, msg):
        """Simulator joint states."""
        self.joint_states = msg
        self.last_joint_time = time.time()
    
    def on_robot_state(self, msg):
        """Robot state for IK feedback."""
        if msg.position:
            self.current_q = np.array(msg.position)
    
    def on_position_cmd(self, msg):
        """Position control commands (IK output)."""
        self.control_cmd = ('position', msg)
        self.last_control_time = time.time()
    
    def on_torque_cmd(self, msg):
        """Torque control commands (impedance output)."""
        self.control_cmd = ('torque', msg)
        self.last_control_time = time.time()
    
    def on_pose_target(self, msg):
        """Target pose."""
        self.pose_target = msg
        
        self.target_pos = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ])
        
        quat_xyzw = np.array([
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        ])
        self.target_rot_mat = Rotation.from_quat(quat_xyzw).as_matrix()
        
        if not self.has_target:
            self.has_target = True
    
    def get_ee_pose(self):
        """Compute current end-effector pose from joint angles."""
        if self.current_q is None or len(self.current_q) < 7:
            return None, None
        
        # Set joint positions and compute forward kinematics
        self.d.qpos[:len(self.current_q)] = self.current_q
        mujoco.mj_forward(self.m, self.d)
        
        # Get EE position and orientation
        pos = self.d.site_xpos[self.ee_site_id].copy()
        rot_mat = self.d.site_xmat[self.ee_site_id].copy().reshape(3, 3)
        
        return pos, rot_mat
    
    def display_status(self):
        """Display comprehensive pipeline status."""
        # Clear screen
        os.system('clear' if os.name == 'posix' else 'cls')
        
        print("=" * 80)
        print("🤖 MuJoCo-ROS2-Control Pipeline Monitor")
        print("=" * 80)
        print(f"⏰ Time: {time.strftime('%H:%M:%S')}")
        print()
        
        # System status
        current_time = time.time()
        sim_status = "🟢 ACTIVE" if self.last_joint_time and (current_time - self.last_joint_time) < 1.0 else "🔴 INACTIVE"
        control_status = "🟢 ACTIVE" if self.last_control_time and (current_time - self.last_control_time) < 1.0 else "🔴 INACTIVE"
        
        print(f"📊 Simulation: {sim_status}")
        print(f"🎮 Control: {control_status}")
        print()
        
        # End-effector tracking
        if self.has_target and self.current_q is not None:
            current_pos, current_rot_mat = self.get_ee_pose()
            
            if current_pos is not None:
                # Position error
                pos_error = np.linalg.norm(current_pos - self.target_pos)
                pos_error_mm = pos_error * 1000
                
                # Orientation error
                rot_diff = self.target_rot_mat @ current_rot_mat.T - np.eye(3)
                ori_error = np.linalg.norm(rot_diff)
                
                # Check if reached
                pos_reached = pos_error < 0.002  # 2mm threshold
                ori_reached = ori_error < 0.02
                
                if pos_reached and ori_reached:
                    status_icon = "✅"
                    status_text = "TARGET REACHED"
                    status_color = "\033[92m"  # Green
                else:
                    status_icon = "🎯"
                    status_text = "MOVING TO TARGET"
                    status_color = "\033[93m"  # Yellow
                
                reset_color = "\033[0m"
                
                print(f"{status_icon} END-EFFECTOR TRACKING: {status_color}{status_text}{reset_color}")
                print(f"   Current:  [{current_pos[0]:+.3f}, {current_pos[1]:+.3f}, {current_pos[2]:+.3f}]")
                print(f"   Target:   [{self.target_pos[0]:+.3f}, {self.target_pos[1]:+.3f}, {self.target_pos[2]:+.3f}]")
                print(f"   Pos Error: {pos_error_mm:6.2f} mm  {'✓' if pos_reached else '→'}")
                print(f"   Ori Error: {ori_error:6.4f}      {'✓' if ori_reached else '→'}")
        elif self.has_target:
            print("🎯 END-EFFECTOR TRACKING:")
            print(f"   Target:   [{self.target_pos[0]:+.3f}, {self.target_pos[1]:+.3f}, {self.target_pos[2]:+.3f}]")
            print(f"   Status: ⏳ Waiting for robot state...")
        else:
            print("🎯 END-EFFECTOR TRACKING: ⏳ Waiting for target pose...")
        
        print()
        
        # Joint states
        if self.joint_states and len(self.joint_states.position) >= 7:
            print("🦾 JOINT STATES (Panda arm):")
            for i in range(min(7, len(self.joint_states.position))):
                name = self.joint_states.name[i] if i < len(self.joint_states.name) else f"joint_{i+1}"
                pos = self.joint_states.position[i]
                vel = self.joint_states.velocity[i] if i < len(self.joint_states.velocity) else 0.0
                
                # Position in degrees
                pos_deg = np.rad2deg(pos)
                
                # Create visual bar for position (-180° to 180°)
                normalized_pos = max(-1, min(1, pos / 3.14159))
                bar_width = 20
                bar_pos = int((normalized_pos + 1) * bar_width / 2)
                bar = "─" * bar_pos + "●" + "─" * (bar_width - bar_pos)
                
                print(f"   {name:12s}: {pos_deg:+7.2f}° │{bar}│ vel:{vel:+.2f}")
        else:
            print("🦾 JOINT STATES: No data")
        
        print()
        
        # Control commands
        if self.control_cmd:
            ctrl_type, msg = self.control_cmd
            
            if ctrl_type == 'position':
                print("📍 CONTROL COMMANDS (Position):")
                if msg.position and len(msg.position) >= 7:
                    for i in range(min(7, len(msg.position))):
                        name = msg.name[i] if i < len(msg.name) else f"joint_{i+1}"
                        pos_cmd = msg.position[i]
                        pos_cmd_deg = np.rad2deg(pos_cmd)
                        
                        # Show velocity if available
                        vel_str = ""
                        if msg.velocity and i < len(msg.velocity):
                            vel = msg.velocity[i]
                            vel_str = f" vel:{vel:+.2f}"
                        
                        print(f"   {name:12s}: {pos_cmd_deg:+7.2f}°{vel_str}")
            
            elif ctrl_type == 'torque':
                print("⚡ CONTROL COMMANDS (Torque):")
                if msg.effort and len(msg.effort) >= 7:
                    total_effort = sum(abs(t) for t in msg.effort[:7])
                    for i in range(min(7, len(msg.effort))):
                        name = msg.name[i] if i < len(msg.name) else f"joint_{i+1}"
                        torque = msg.effort[i]
                        
                        # Visual torque bar
                        max_torque = 10.0
                        normalized_torque = max(-1, min(1, torque / max_torque))
                        bar_width = 15
                        if abs(normalized_torque) < 0.05:
                            bar = "─" * (bar_width//2) + "●" + "─" * (bar_width//2)
                        else:
                            bar_pos = int(abs(normalized_torque) * bar_width / 2)
                            if normalized_torque > 0:
                                bar = "─" * (bar_width//2) + "●" + "█" * bar_pos + "─" * (bar_width//2 - bar_pos)
                            else:
                                bar = "─" * (bar_width//2 - bar_pos) + "█" * bar_pos + "●" + "─" * (bar_width//2)
                        
                        print(f"   {name:12s}: {torque:+6.3f} N⋅m │{bar}│")
                    print(f"   Total effort: {total_effort:.2f} N⋅m")
        else:
            print("🎮 CONTROL COMMANDS: No data")
        
        print()
        print("=" * 80)
        print("💡 Press Ctrl+C to exit")


def main():
    rclpy.init()
    
    try:
        monitor = PipelineMonitor()
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        print("\n\n👋 Monitor stopped.")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()

