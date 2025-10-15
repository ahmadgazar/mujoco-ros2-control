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

        mjcf_path = self.get_parameter('mjcf_path').value
        if self.get_parameter('use_robot_descriptions').value and not mjcf_path:
            try:
                from robot_descriptions import panda_mj_description
                mjcf_path = panda_mj_description.MJCF_PATH
            except Exception as e:
                self.get_logger().error(f"Failed to load robot_descriptions: {e}")
                raise

        self.m = mujoco.MjModel.from_xml_path(mjcf_path)
        self.d = mujoco.MjData(self.m)
        self.conf = mink.Configuration(self.m)

        self.ee_frame = self.get_parameter('end_effector_frame').value
        self.ee_frame_type = self.get_parameter('end_effector_frame_type').value

        jn = self.get_parameter('joint_names').value
        self.joint_names = list(jn) if jn and jn[0] else [
            self.m.joint(i).name for i in range(self.m.njnt)
        ]

        self.tasks = [mink.tasks.FrameTask(
            frame_name=self.ee_frame,
            frame_type=self.ee_frame_type,
            position_cost=1.0,
            orientation_cost=1.0,
        )]
        self.last_q = np.copy(self.d.qpos)
        self.current_q = None  # Actual joint positions from simulator/robot

        self.create_subscription(PoseStamped, '/teleop/pose_target', self.on_target, 10)
        self.create_subscription(JointState, '/robot/state', self.on_state, 200)
        self.pub = self.create_publisher(JointState, '/ctrl/q_ref', 10)
        self.create_timer(float(self.get_parameter('ik_dt').value), self.solve_ik)

    def on_target(self, msg: PoseStamped):
        """Set IK target from pose message."""
        pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        quat = np.array([
            msg.pose.orientation.x, msg.pose.orientation.y,
            msg.pose.orientation.z, msg.pose.orientation.w
        ])
        rot = Rotation.from_quat(quat).as_matrix()
        transform = mink.SE3.from_rotation_and_translation(mink.SO3.from_matrix(rot), pos)
        self.tasks[0].set_target(transform)
    
    def on_state(self, msg: JointState):
        """Receive actual joint states from simulator/robot."""
        if msg.position:
            self.current_q = np.array(msg.position)

    def solve_ik(self):
        """Solve IK and publish joint references."""
        if self.tasks[0].transform_target_to_world is None:
            return
        
        # Use actual joint positions if available, otherwise use integrated state
        if self.current_q is not None:
            self.conf.q[:len(self.current_q)] = self.current_q
        
        dt = float(self.get_parameter('ik_dt').value)
        dq = mink.solve_ik(self.conf, self.tasks, dt=dt, solver='daqp', damping=1e-6)
        
        vmax = float(self.get_parameter('vel_limit').value)
        dq = np.clip(dq[:self.m.nv], -vmax, vmax)
        
        q_d = self.conf.q.copy()
        q_d[:dq.size] += dq * dt
        self.last_q = q_d
        
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = q_d[:len(self.joint_names)].tolist()
        msg.velocity = dq[:len(self.joint_names)].tolist()
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = MinkIKNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
