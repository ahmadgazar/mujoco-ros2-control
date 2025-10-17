#!/usr/bin/env python3
"""MINK differential inverse kinematics solver for ROS 2."""

import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
import mujoco
import mink
from scipy.spatial.transform import Rotation


class MinkIKNode(Node):
    """Solves IK using MINK differential IK to track Cartesian targets."""
    
    def __init__(self):
        super().__init__('mink_ik')

        self.declare_parameter('use_robot_descriptions', True)
        self.declare_parameter('robot_description_pkg', 'panda_mj_description')
        self.declare_parameter('variant', '')
        self.declare_parameter('mjcf_path', '')
        self.declare_parameter('end_effector_frame', 'hand')
        self.declare_parameter('end_effector_frame_type', 'body')
        self.declare_parameter('joint_names', [''])
        self.declare_parameter('ik_dt', 0.005)
        self.declare_parameter('vel_limit', 2.0)
        self.declare_parameter('solver_iter_limit', 100)

        mjcf_path = self.get_parameter('mjcf_path').value
        if self.get_parameter('use_robot_descriptions').value and not mjcf_path:
            try:
                from robot_descriptions import panda_mj_description
                mjcf_path = panda_mj_description.MJCF_PATH
            except Exception as e:
                self.get_logger().error(f"Failed to load robot_descriptions: {e}")
                
                # Check for common Git lock file issue and provide helpful guidance
                if "index.lock" in str(e) or "Another git process seems to be running" in str(e):
                    self.get_logger().error(
                        "Git lock file detected. Try running: "
                        "rm -f ~/.cache/robot_descriptions/*/mujoco_menagerie/.git/index.lock"
                    )
                
                self.get_logger().error("Falling back to manual MJCF path specification required.")
                self.get_logger().error("Set 'mjcf_path' parameter or fix robot_descriptions cache.")
                raise

        self.m = mujoco.MjModel.from_xml_path(mjcf_path)
        self.d = mujoco.MjData(self.m)
        
        # Load home keyframe if available (following official mink example pattern)
        if self.m.nkey > 0:
            home_key_id = self.m.key("home").id
            mujoco.mj_resetDataKeyframe(self.m, self.d, home_key_id)
            self.get_logger().info(f"Loaded 'home' keyframe for IK initial state")
            self.get_logger().info(f"Initial qpos (deg): {np.rad2deg(self.d.qpos[:7])}")
        
        # Create configuration and initialize from data (like official example)
        self.conf = mink.Configuration(self.m)
        self.conf.update(self.d.qpos)
        
        # Store home position for verification
        self.home_q = self.d.qpos[:7].copy()
        self.get_logger().info(f"Home configuration: {np.rad2deg(self.home_q)}")

        self.ee_frame = self.get_parameter('end_effector_frame').value
        self.ee_frame_type = self.get_parameter('end_effector_frame_type').value

        jn = self.get_parameter('joint_names').value
        self.joint_names = list(jn) if jn and jn[0] else [
            self.m.joint(i).name for i in range(self.m.njnt)
        ]

        # Create tasks (following official mink example - with PostureTask for regularization)
        end_effector_task = mink.tasks.FrameTask(
            frame_name=self.ee_frame,
            frame_type=self.ee_frame_type,
            position_cost=1.0,
            orientation_cost=1.0,
            lm_damping=1.0,
        )
        posture_task = mink.tasks.PostureTask(model=self.m, cost=1e-2)
        posture_task.set_target_from_configuration(self.conf)
        
        self.tasks = [end_effector_task, posture_task]
        self.get_logger().info("Created FrameTask and PostureTask (regularization)")
        self.last_q = np.copy(self.d.qpos)
        self.current_q = None    # Actual joint positions from simulator/robot (only updated when state feedback arrives)
        self.is_at_home = False  # Flag to track if robot is at home position
        
        # Track startup time to ignore stale targets
        self.startup_time = self.get_clock().now()
        self.min_startup_duration_sec = 7.5  # Ignore targets for first 7.5 seconds

        self.create_subscription(PoseStamped, '/teleop/pose_target', self.on_target, 10)
        self.create_subscription(JointState, '/robot/state', self.on_state, 200)  # Via router from /mujoco/joint_states
        self.get_logger().info("Subscribed to /robot/state for state feedback")
        self.pub = self.create_publisher(JointState, '/ctrl/q_ref', 10)
        self.create_timer(float(self.get_parameter('ik_dt').value), self.solve_ik)

    def on_target(self, msg: PoseStamped):
        """Set IK target from pose message."""
        # Safety: Ignore targets received too soon after startup (may be stale from DDS cache)
        elapsed_startup = (self.get_clock().now() - self.startup_time).nanoseconds / 1e9
        if elapsed_startup < self.min_startup_duration_sec:
            return
        
        # Safety: Ignore targets until robot is verified at home position
        if not self.is_at_home:
            return
        
        pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        quat = np.array([
            msg.pose.orientation.x, msg.pose.orientation.y,
            msg.pose.orientation.z, msg.pose.orientation.w
        ])
        rot = Rotation.from_quat(quat).as_matrix()
        transform = mink.SE3.from_rotation_and_translation(mink.SO3.from_matrix(rot), pos)
        self.tasks[0].set_target(transform)
        
        # Log first target accepted
        if not hasattr(self, '_target_received'):
            self._target_received = True
            self.get_logger().info(f"Target pose accepted: {pos}")
    
    def on_state(self, msg: JointState):
        """Receive actual joint states from simulator/robot (via router for cache-free data)."""
        if msg.position:
            # Count messages and wait for stabilization
            if not hasattr(self, '_state_msg_count'):
                self._state_msg_count = 0
                self.get_logger().info("Robot state feedback connected")
            self._state_msg_count += 1
            
            self.current_q = np.array(msg.position)
            
            # Wait for sim to stabilize (~0.1 second)
            if self._state_msg_count < 50:
                return
            
            # Check if robot is at home position (within tolerance)
            if not self.is_at_home and len(self.current_q) >= 7:
                pos_error = np.linalg.norm(self.current_q[:7] - self.home_q)
                
                # Use 0.10 rad (~5.7 deg) tolerance for simulation settling
                if pos_error < 0.10:
                    self.is_at_home = True
                    self.get_logger().info(f"Robot at home position (error: {pos_error:.4f} rad)")
                elif self._state_msg_count % 100 == 0:
                    self.get_logger().warn(f"Waiting for robot at home (error: {pos_error:.4f} rad)")

    def solve_ik(self):
        """Solve IK and publish joint references."""
        # Safety checks before solving IK
        if self.tasks[0].transform_target_to_world is None:
            return  # No target yet
        
        if self.current_q is None:
            return  # No state feedback yet
        
        if not self.is_at_home:
            return  # Robot not at home yet
        
        # Log when IK starts (once)
        if not hasattr(self, '_ik_started'):
            self._ik_started = True
            self.get_logger().info("IK solver started")
        
        # Update configuration from actual joint positions
        self.conf.update(self.current_q)
        
        dt = float(self.get_parameter('ik_dt').value)
        
        # Solve IK (using daqp solver)
        vel = mink.solve_ik(self.conf, self.tasks, dt, "daqp", 1e-3)
        
        vmax = float(self.get_parameter('vel_limit').value)
        vel = np.clip(vel[:self.m.nv], -vmax, vmax)
        
        # Compute desired position as current + velocity*dt
        # This prevents accumulation when feedback is delayed
        q_d = self.current_q.copy()
        q_d[:len(vel)] += vel * dt
        self.last_q = q_d
        
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ''  # Empty string must be explicitly set for proper serialization
        msg.name = [str(name) for name in self.joint_names]  # Ensure proper string types
        msg.position = q_d[:len(self.joint_names)].tolist()
        msg.velocity = vel[:len(self.joint_names)].tolist()
        
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = MinkIKNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
