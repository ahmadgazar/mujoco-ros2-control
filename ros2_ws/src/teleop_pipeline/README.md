# Teleop Pipeline

ROS 2 package implementing a teleoperation control pipeline with MINK inverse kinematics, impedance control, and MuJoCo simulation.

## Nodes

### Core Nodes
- **pose_target_pub**: Target pose publisher with configurable startup delay
- **mink_ik_node**: MINK differential IK solver with safety checks
- **impedance_node**: Joint-space impedance controller  
- **torque_router**: Message router (breaks DDS caching, required for consistent behavior)
- **mujoco_sim_bridge**: MuJoCo physics simulator with integrated viewer support

### Optional
- **mujoco_visualizer**: Standalone visualization node (deprecated, use integrated viewer)

## Quick Start

```bash
# Build (use pixi for proper environment)
pixi run build

# Launch with torque control (full impedance pipeline)
pixi run launch-viewer

# OR launch with direct position control (IK only, faster)
pixi run test-ik
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
| `/ctrl/q_ref` | JointState | Desired joint positions from IK |
| `/ctrl/torques` | JointState | Control torques from impedance controller |
| `/mujoco/joint_states` | JointState | Raw simulation state (internal) |
| `/mujoco/effort_command` | JointState | Torque commands to simulator |
| `/robot/state` | JointState | Current robot state (via router, cache-free) |

**Note**: `/robot/state` is republished by `torque_router` from `/mujoco/joint_states` with fresh timestamps to prevent DDS caching issues.

## See Also

See main [README](../../../README.md) for complete documentation.
