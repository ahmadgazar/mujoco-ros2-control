# IK-MuJoCo-ROS2

A ROS 2 teleoperation pipeline integrating MINK differential inverse kinematics, impedance control, and MuJoCo physics simulation for robotic manipulation.

## Overview

This project implements a complete control pipeline:
1. **Target Generation** → Target end-effector poses
2. **Inverse Kinematics** → Joint configurations using MINK differential IK
3. **Impedance Control** → Compliant torque commands
4. **Physics Simulation** → MuJoCo-based robot simulation

## Features

- **MINK IK Solver**: Real-time differential inverse kinematics with velocity limiting
- **Impedance Controller**: PD-based joint-space impedance control
- **MuJoCo Integration**: High-fidelity physics simulation
- **Modular Architecture**: ROS2 node-based design
- **Robot Descriptions**: Automatic model loading from `robot_descriptions` package

## Requirements

- **Python**: 3.10+
- **ROS 2**: Humble (via RoboStack)
- **Pixi**: Package manager for environment management

## Installation

### 1. Install Pixi

```bash
curl -fsSL https://pixi.sh/install.sh | bash
```

### 2. Clone Repository

```bash
git clone https://github.com/yourusername/IK-mujco-ros2.git
cd IK-mujco-ros2
```

### 3. Install Dependencies

Pixi automatically manages all dependencies including ROS 2, MuJoCo, and Python packages:

```bash
pixi install
```

## Building

Build the ROS 2 workspace:

```bash
pixi run build
```

## Usage

### Launch the Complete Pipeline

```bash
pixi run launch
```

This starts all nodes:
- `pose_target_pub`: Publishes target poses
- `mink_ik_node`: Computes inverse kinematics
- `impedance_node`: Generates control torques
- `torque_router`: Routes messages between nodes
- `mujoco_sim_bridge`: Runs physics simulation

### Monitor Simulation

```bash
# View joint states
ros2 topic echo /mujoco/joint_states

# Check control torques
ros2 topic echo /ctrl/torques

# Monitor target poses
ros2 topic echo /teleop/pose_target

# List all topics
ros2 topic list
```

## Architecture

### Nodes

#### `pose_target_pub`
Publishes target end-effector poses.

**Parameters:**
TODO: dynamically update the reference - fixed parameter for now
- `position`: Target position [x, y, z] (default: [0.5, 0.0, 0.4])
- `orientation_xyzw`: Target orientation as quaternion (default: [0, 0, 0, 1])

**Publishes:**
- `/teleop/pose_target` (PoseStamped): Target end-effector pose

#### `mink_ik_node`
Solves inverse kinematics using MINK differential IK.

**Parameters:**
- `use_robot_descriptions`: Load model from robot_descriptions (default: true)
- `robot_description_pkg`: Package name (default: 'panda_mj_description')
- `end_effector_frame`: End-effector frame name (default: 'hand')
- `end_effector_frame_type`: Frame type ('body', 'site', or 'geom')
- `joint_names`: List of joint names to control
- `ik_dt`: IK solver timestep in seconds (default: 0.005)
- `vel_limit`: Maximum joint velocity (default: 2.0 rad/s)

**Subscribes:**
- `/teleop/pose_target` (PoseStamped): Target poses
- `/robot/state` (JointState): Current joint states for feedback

**Publishes:**
- `/ctrl/q_ref` (JointState): Desired joint positions and velocities

#### `impedance_node`
Computes control torques using impedance control law: τ = K(q_d - q) + D(q̇_d - q̇)

**Parameters:**
- `joint_names`: List of joint names
- `k_diag`: Stiffness diagonal values (N⋅m/rad)
- `d_diag`: Damping diagonal values (N⋅m⋅s/rad)

**Subscribes:**
- `/ctrl/q_ref` (JointState): Desired joint states from IK
- `/robot/state` (JointState): Current joint states

**Publishes:**
- `/ctrl/torques` (JointState): Control torques

#### `torque_router`
Routes messages between controller and simulator.

**Subscribes:**
- `/ctrl/torques` (JointState): Control torques from impedance controller
- `/mujoco/joint_states` (JointState): Joint states from simulator

**Publishes:**
- `/mujoco/effort_command` (JointState): Torque commands to simulator
- `/robot/state` (JointState): Joint states to controller

#### `mujoco_sim_bridge`
Runs MuJoCo physics simulation.

**Parameters:**
- `use_robot_descriptions`: Load model from robot_descriptions (default: true)
- `robot_description_pkg`: Package name (default: 'panda_mj_description')
- `sim_dt`: Simulation timestep in seconds (default: 0.002)
- `use_viewer`: Enable MuJoCo viewer (default: false)

**Subscribes:**
- `/mujoco/effort_command` (JointState): Control torques

**Publishes:**
- `/mujoco/joint_states` (JointState): Current joint states

### Topic Graph

```
                    ┌─────────────────┐
                    │ pose_target_pub │
                    └────────┬────────┘
                             │ /teleop/pose_target
                             ▼
                    ┌─────────────────┐
            ┌──────►│  mink_ik_node   │
            │       └────────┬────────┘
            │                │ /ctrl/q_ref
            │                ▼
            │       ┌─────────────────┐
            │  ┌───►│ impedance_node  │
            │  │    └────────┬────────┘
            │  │             │ /ctrl/torques
            │  │             ▼
            │  │    ┌─────────────────┐
            │  │    │  torque_router  │◄──┐
            │  │    └────────┬────────┘   │
            │  │             │ /mujoco/effort_command
            │  │             ▼              │
            │  │    ┌──────────────────┐   │
            │  │    │ mujoco_sim_bridge│   │
            │  │    └────────┬─────────┘   │
            │  │             │ /mujoco/joint_states
            │  │             └──────────────┘
            │  │             │
            │  └─────────────┴── /robot/state
            │                │
            └────────────────┘
```

## Configuration

### Default Robot: Franka Panda

The pipeline is pre-configured for the Franka Emika Panda robot with 7-DOF arm control. Configuration is in `ros2_ws/src/teleop_pipeline/launch/teleop_mujoco.launch.py`.

### Custom Robots

To use a different robot:

1. Ensure your robot model is available in `robot_descriptions` or provide a MuJoCo XML path
2. Update launch file with:
   - Correct joint names
   - End-effector frame name and type
   - Appropriate stiffness/damping gains

Example for different robot:

```python
custom_joints = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]

Node(
    package='teleop_pipeline',
    executable='mink_ik_node',
    name='mink_ik',
    parameters=[{
        'mjcf_path': '/path/to/your/robot.xml',
        'use_robot_descriptions': False,
        'end_effector_frame': 'end_effector',
        'end_effector_frame_type': 'site',
        'joint_names': custom_joints
    }]
)
```

## Pixi Tasks

Convenient task commands defined in `pixi.toml`:

```bash
pixi run build         # Build ROS 2 workspace
pixi run launch        # Launch full pipeline
pixi run shell         # Open shell in Pixi environment
```

To run individual nodes for debugging:

```bash
# Source the environment first
pixi shell

# Then run individual nodes
ros2 run teleop_pipeline impedance_node
ros2 run teleop_pipeline mink_ik_node
# etc.
```

## Known Limitations

- **macOS Viewer**: MuJoCo GUI viewer requires `mjpython` which is incompatible with ROS 2. Run simulation headless and monitor via ROS topics.
- **Platform**: Optimized for macOS ARM64 and Linux. Windows support untested.

## Troubleshooting

### Build Errors

```bash
# Clean build
rm -rf ros2_ws/build ros2_ws/install ros2_ws/log
pixi run build
```

### Runtime Issues

```bash
# Check node status
ros2 node list

# Check topic connections
ros2 topic info /ctrl/torques

# Monitor topic rates
ros2 topic hz /mujoco/joint_states
```

### Cache Issues

If robot model loading fails:

```bash
rm -rf ~/.cache/robot_descriptions
```

## Development

### Project Structure

```
IK-mujco-ros2/
├── pixi.toml                   # Pixi environment configuration
├── ros2_ws/                    # ROS 2 workspace
│   └── src/
│       └── teleop_pipeline/    # Main package
│           ├── teleop_pipeline/          # Python nodes
│           │   ├── mink_ik_node.py       # IK solver
│           │   ├── impedance_node.py     # Impedance controller
│           │   ├── mujoco_sim_bridge.py  # MuJoCo simulator
│           │   ├── torque_router.py      # Message router
│           │   └── pose_target_pub.py    # Target publisher
│           ├── launch/                   # Launch files
│           │   └── teleop_mujoco.launch.py
│           ├── setup.py                  # Package setup
│           └── package.xml               # Package metadata
└── README.md
```

### Adding New Nodes

1. Create node in `ros2_ws/src/teleop_pipeline/teleop_pipeline/`
2. Add entry point in `setup.py`:

```python
entry_points={
    'console_scripts': [
        'your_node = teleop_pipeline.your_node:main',
    ],
},
```

3. Rebuild: `pixi run build`

## Dependencies

### Core
- **ROS 2 Humble**: Robot Operating System
- **MuJoCo 3.x**: Physics simulation
- **MINK**: Differential inverse kinematics solver
- **robot_descriptions**: Robot model repository

### Python
- `numpy`: Numerical computing
- `scipy`: Scientific computing
- `rclpy`: ROS 2 Python client library

All dependencies are automatically managed by Pixi.

## License

[TODO: Add license here]

## Acknowledgments

- [MINK](https://github.com/stephane-caron/mink): Differential inverse kinematics library
- [MuJoCo](https://mujoco.org/): Multi-Joint dynamics with Contact
- [RoboStack](https://robostack.github.io/): Conda packages for ROS
- [robot_descriptions](https://github.com/robot-descriptions/robot_descriptions.py): Robot models repository

