# Teleop Pipeline

ROS 2 package implementing a teleoperation control pipeline with MINK inverse kinematics, impedance control, and MuJoCo simulation.

## Nodes

### Core Nodes
- **pose_target_pub**: Target pose publisher
- **mink_ik_node**: MINK differential IK solver
- **impedance_node**: Joint-space impedance controller  
- **torque_router**: Message router
- **mujoco_sim_bridge**: MuJoCo physics simulator

### Optional
- **mujoco_visualizer**: Standalone visualization node (macOS incompatible with ROS 2)

## Quick Start

```bash
# Build
colcon build --symlink-install

# Launch
ros2 launch teleop_pipeline teleop_mujoco.launch.py
```

## Configuration

Edit `launch/teleop_mujoco.launch.py` to:
- Change target positions
- Adjust impedance gains
- Configure different robots
- Modify IK parameters

## Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/teleop/pose_target` | PoseStamped | Target end-effector pose |
| `/ctrl/q_ref` | JointState | Desired joint positions |
| `/ctrl/torques` | JointState | Control torques |
| `/mujoco/joint_states` | JointState | Simulation state |
| `/mujoco/effort_command` | JointState | Torque commands to simulator |
| `/robot/state` | JointState | Current robot state |

## See Also

See main [README](../../../README.md) for complete documentation.
