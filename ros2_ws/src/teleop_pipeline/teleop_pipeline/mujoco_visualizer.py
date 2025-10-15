#!/usr/bin/env python3
"""
Standalone MuJoCo visualizer that subscribes to /mujoco/joint_states
Run this on your HOST MACHINE (not in container) to visualize the simulation.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import mujoco
import mujoco.viewer
import time

#TODO: integrate with the ROS2 pipeline 
class MuJoCoVisualizer(Node):
    def __init__(self):
        super().__init__('mujoco_visualizer')
        
        # Parameters
        self.declare_parameter('use_robot_descriptions', True)
        self.declare_parameter('robot_description_pkg', 'panda_mj_description')
        
        # Load model (same as sim)
        if self.get_parameter('use_robot_descriptions').value:
            try:
                from robot_descriptions import panda_mj_description
                mjcf_path = panda_mj_description.MJCF_PATH
                self.get_logger().info(f"Loaded model: {mjcf_path}")
            except Exception as e:
                self.get_logger().error(f"Failed to load robot_descriptions: {e}")
                return
        
        self.m = mujoco.MjModel.from_xml_path(mjcf_path)
        self.d = mujoco.MjData(self.m)
        
        # Get joint names
        self.joint_names = []
        for i in range(self.m.njnt):
            jnt_type = self.m.jnt_type[i]
            if jnt_type != mujoco.mjtJoint.mjJNT_FREE:
                self.joint_names.append(self.m.joint(i).name)
        
        self.get_logger().info(f"Visualizing joints: {self.joint_names}")
        
        # Subscribe to joint states from simulator
        self.sub = self.create_subscription(
            JointState, '/mujoco/joint_states', self.on_joint_state, 10)
        
        self.get_logger().info("MuJoCo visualizer ready. Launch viewer manually with mujoco.viewer.launch()")
        
    def on_joint_state(self, msg: JointState):
        """Update model data from joint states"""
        if len(msg.position) > 0:
            n = min(len(msg.position), len(self.d.qpos))
            self.d.qpos[:n] = msg.position[:n]
        if len(msg.velocity) > 0:
            n = min(len(msg.velocity), len(self.d.qvel))
            self.d.qvel[:n] = msg.velocity[:n]
        
        # Forward kinematics to update visualization
        mujoco.mj_forward(self.m, self.d)

def main():
    import threading
    rclpy.init()
    
    try:
        node = MuJoCoVisualizer()
        
        # Run ROS spinner in a background thread
        def ros_spin():
            rclpy.spin(node)
        
        spin_thread = threading.Thread(target=ros_spin, daemon=True)
        spin_thread.start()
        
        # Run MuJoCo viewer in main thread (required for macOS)
        node.get_logger().info("Starting MuJoCo viewer in main thread...")
        with mujoco.viewer.launch_passive(node.m, node.d) as viewer:
            while viewer.is_running():
                viewer.sync()
                time.sleep(0.01)
        
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Viewer error: {e}")
        # Fall back to non-viewer mode
        node.get_logger().warn("Viewer failed, running headless")
        rclpy.spin(node)
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()

