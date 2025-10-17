#!/usr/bin/env python3
"""MuJoCo simulation bridge for ROS 2."""

import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import JointState
import mujoco
import mujoco.viewer
import threading
import time
import copy


class MuJoCoSimBridge(Node):
    """Runs MuJoCo physics simulation and publishes joint states."""
    
    def __init__(self):
        super().__init__('mujoco_sim_bridge')
        
        self.declare_parameter('mjcf_path', '')
        self.declare_parameter('use_robot_descriptions', True)
        self.declare_parameter('robot_description_pkg', 'panda_mj_description')
        self.declare_parameter('sim_dt', 0.002)
        self.declare_parameter('use_viewer', False)
        self.declare_parameter('control_mode', 'torque')  # 'torque' or 'position'
        
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
        
        # Initialize simulation properly to prevent instabilities
        mujoco.mj_resetData(self.m, self.d)
        
        # Load home keyframe if available
        if self.m.nkey > 0:
            home_key_id = self.m.key("home").id
            mujoco.mj_resetDataKeyframe(self.m, self.d, home_key_id)
            self.d.ctrl[:] = self.m.key_ctrl[home_key_id]  # Initialize control
            self.get_logger().info(f"Loaded 'home' keyframe as initial configuration")
        else:
            self.get_logger().warn(f"No keyframes in model from: {mjcf_path}")
        
        mujoco.mj_forward(self.m, self.d)
        
        # Stabilize simulation
        for _ in range(10):
            mujoco.mj_step1(self.m, self.d)
            mujoco.mj_step2(self.m, self.d)
        
        self.joint_names = [
            self.m.joint(i).name for i in range(self.m.njnt)
            if self.m.jnt_type[i] != mujoco.mjtJoint.mjJNT_FREE
        ]
        self.get_logger().info(f"Simulating joints: {self.joint_names}")
        
        self.control_mode = self.get_parameter('control_mode').value
        self.get_logger().info(f"Control mode: {self.control_mode}")
        
        # Store initial ctrl values to maintain during startup
        self.desired_ctrl = self.d.ctrl.copy()
        
        # Flag to prevent publishing before initialization is complete
        self.initialized = False
        self._init_steps = 0
        
        # Publish to MuJoCo's own topic - router will forward to /robot/state
        self.pub_state = self.create_publisher(JointState, '/mujoco/joint_states', 10)
        self.get_logger().info("Publishing joint states to /mujoco/joint_states")
        
        # Subscribe to appropriate command topic based on control mode
        if self.control_mode == 'position':
            self.sub_cmd = self.create_subscription(JointState, '/ctrl/q_ref', self.on_position_cmd, 10)
            self.get_logger().info("Subscribed to /ctrl/q_ref for position control")
        else:
            self.sub_cmd = self.create_subscription(JointState, '/mujoco/effort_command', self.on_effort_cmd, 10)
            self.get_logger().info("Subscribed to /mujoco/effort_command for torque control")
        
        dt = float(self.get_parameter('sim_dt').value)
        self.m.opt.timestep = dt
        
        # Additional stabilization before starting timer
        for _ in range(30):
            mujoco.mj_step(self.m, self.d)
        self.initialized = True
        
        self.create_timer(dt, self.sim_step)
        
        # Initialize threading resources BEFORE starting viewer thread
        self.effort_cmd = np.zeros(self.m.nu)
        self.data_lock = threading.Lock()  # Protect shared data access
        self.viewer_active = False
        
        if self.get_parameter('use_viewer').value:
            try:
                import os
                
                # Enhanced Linux/Wayland compatibility setup
                self.get_logger().info("Configuring display environment for MuJoCo viewer...")
                
                # Use default GLFW rendering (best compatibility with hardware)
                # Don't force OSMesa unless explicitly set
                if 'MUJOCO_GL' not in os.environ:
                    os.environ['MUJOCO_GL'] = 'glfw'
                
                # Wayland compatibility - force X11 backend
                if 'WAYLAND_DISPLAY' in os.environ:
                    self.get_logger().info("Detected Wayland, configuring for X11 compatibility...")
                    os.environ['GDK_BACKEND'] = 'x11'
                    os.environ['QT_QPA_PLATFORM'] = 'xcb'
                    os.environ['SDL_VIDEODRIVER'] = 'x11'
                
                # GLFW configuration for better stability
                os.environ['GLFW_IM_MODULE'] = 'ibus'
                
                # Disable VSync for better performance
                os.environ['__GL_SYNC_TO_VBLANK'] = '0'
                
                self.get_logger().info("Starting MuJoCo viewer...")
                
                # NOTE: Do NOT reset data here - keyframe was already loaded above!
                # The viewer thread will use the already-configured state
                
                viewer_thread = threading.Thread(
                    target=self._run_viewer_safe,
                    daemon=True
                )
                viewer_thread.start()
                
            except Exception as e:
                self.get_logger().error(f"Failed to start MuJoCo viewer: {e}")
                self.get_logger().info("Continuing in headless mode...")
    
    def _run_viewer_safe(self):
        """Run MuJoCo viewer in a safe threaded manner."""
        try:
            self.viewer_active = True
            self.get_logger().info("Starting viewer...")
            
            # Use context manager to ensure proper cleanup
            with mujoco.viewer.launch_passive(self.m, self.d) as viewer:
                self.get_logger().info("Viewer ready")
                
                # Configure camera and rendering options for better initial view
                try:
                    # Enable visualization of the robot
                    viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_JOINT] = True
                    viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_ACTUATOR] = True
                    
                    # Set camera to look at the robot from a good angle
                    viewer.cam.azimuth = 90  # Rotate view
                    viewer.cam.elevation = -15  # Tilt down slightly
                    viewer.cam.distance = 2.0  # Distance from center
                    viewer.cam.lookat[:] = [0.3, 0.0, 0.3]  # Look at robot workspace
                    
                    # Ensure proper rendering
                    with self.data_lock:
                        viewer.sync()  # Initial sync to render the robot
                    
                except Exception as cam_e:
                    pass  # Camera configuration is optional
                
                # Give window time to initialize and render
                time.sleep(0.2)
                
                frame_count = 0
                consecutive_errors = 0
                
                # Keep viewer running until manually closed or stopped
                while self.viewer_active:
                    try:
                        # Sync viewer with simulation data first
                        # This updates the viewer and keeps it alive
                        # Use lock to prevent concurrent access with mj_step
                        with self.data_lock:
                            viewer.sync()
                        
                        # Check if viewer window is still open
                        if not viewer.is_running():
                            break
                        
                        # Reset error counter on successful sync
                        consecutive_errors = 0
                        
                        # Log status periodically
                        frame_count += 1
                        if frame_count % 600 == 0:  # Every ~10 seconds at 60fps
                            self.get_logger().info(f"Viewer active (frame {frame_count})")
                        
                        # Control frame rate
                        time.sleep(0.016)  # ~60 FPS
                        
                    except Exception as sync_e:
                        consecutive_errors += 1
                        self.get_logger().warn(f"Viewer sync issue ({consecutive_errors}): {sync_e}")
                        
                        if consecutive_errors > 10:
                            self.get_logger().error("Too many consecutive viewer errors, stopping viewer")
                            break
                        
                        time.sleep(0.1)  # Brief pause on error
                        
        except Exception as e:
            self.get_logger().error(f"Viewer error: {e}")
            import traceback
            self.get_logger().error(f"Traceback: {traceback.format_exc()}")
        finally:
            self.viewer_active = False            
            self.get_logger().info("Viewer stopped")
        
    def on_position_cmd(self, msg: JointState):
        """Receive position commands from IK (position control mode)."""
        if msg.position and len(msg.position) > 0:
            with self.data_lock:
                n = min(len(msg.position), self.m.nu)
                self.desired_ctrl[:n] = msg.position[:n]
    
    def on_effort_cmd(self, msg: JointState):
        """Receive torque commands (torque control mode)."""
        if msg.effort and len(msg.effort) > 0:
            with self.data_lock:  # Protect against viewer access
                n = min(len(msg.effort), self.m.nu)
                self.effort_cmd[:n] = msg.effort[:n]
    
    def sim_step(self):
        """Step simulation and publish state."""
        try:
            # Apply control and step simulation
            # Use lock to prevent viewer from accessing data during physics step
            with self.data_lock:
                # Apply desired control based on mode
                if self.control_mode == 'position':
                    self.d.ctrl[:] = self.desired_ctrl
                else:  # torque mode
                    self.d.ctrl[:] = self.effort_cmd
                
                mujoco.mj_step(self.m, self.d)
            
            # Wait for initialization before publishing
            if not self.initialized:
                self._init_steps += 1
                if self._init_steps >= 20:
                    self.initialized = True
                return
            
            # Only create and publish if there are subscribers (avoid serialization warnings)
            if self.pub_state.get_subscription_count() > 0:
                # Create message with safe data copy
                js = JointState()
                js.header.stamp = self.get_clock().now().to_msg()
                js.header.frame_id = ''  # Explicitly set empty frame_id to avoid serialization issues
                
                with self.data_lock:
                    # CRITICAL: Ensure names, positions, velocities, efforts all have SAME length
                    # Use actual qpos/qvel length, not joint_names length (handles mimic joints)
                    actual_dof = min(len(self.d.qpos), len(self.d.qvel), len(self.joint_names))
                    
                    # Trim joint names to match actual DOF
                    js.name = [str(name) for name in self.joint_names[:actual_dof]]
                    js.position = self.d.qpos[:actual_dof].tolist()
                    js.velocity = self.d.qvel[:actual_dof].tolist() 
                    js.effort = self.d.qfrc_actuator[:actual_dof].tolist()
            
                # Publish to /mujoco/joint_states - router will forward to /robot/state
                self.pub_state.publish(js)
            
        except Exception as e:
            self.get_logger().error(f"Simulation step error: {e}")
            # Reset simulation if it becomes unstable
            mujoco.mj_resetData(self.m, self.d)
            mujoco.mj_forward(self.m, self.d)

def main():
    rclpy.init()
    node = MuJoCoSimBridge()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

