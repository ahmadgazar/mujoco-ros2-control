from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node

def generate_launch_description():
    panda_joints = [
        "panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4",
        "panda_joint5", "panda_joint6", "panda_joint7"
    ]
    
    return LaunchDescription([
        SetEnvironmentVariable('ROBOT_DESCRIPTION_COMMIT', 'main'),
        
        Node(
            package='teleop_pipeline',
            executable='pose_target_pub',
            name='pose_target_pub',
            output='screen',
            parameters=[{
                'position': [0.594, 0.050, 0.565],  
                'orientation_xyzw': [0.0, 0.707, 0.707, 0.0],  
                'startup_delay': 8.0  
            }]
        ),
        
        Node(
            package='teleop_pipeline',
            executable='mink_ik_node',
            name='mink_ik',
            output='screen',
            parameters=[{
                'use_robot_descriptions': False,
                'mjcf_path': '/home/ahmad.gazar/devel/mujoco-ros2-control/models/franka_panda/mjx_scene.xml',
                'end_effector_frame': 'attachment_site',
                'end_effector_frame_type': 'site',
                'joint_names': panda_joints,
                'ik_dt': 0.02,  
                'vel_limit': 0.5,  
                'solver_iter_limit': 20
            }]
        ),
        
        # Router - breaks DDS cache by republishing messages
        Node(
            package='teleop_pipeline',
            executable='torque_router',
            name='router',
            output='screen'
        ),
        
        # MuJoCo simulation with POSITION CONTROL (no impedance node)
        Node(
            package='teleop_pipeline',
            executable='mujoco_sim_bridge',
            name='mujoco_sim',
            output='screen',
            parameters=[{
                'use_robot_descriptions': False,
                'mjcf_path': '/home/ahmad.gazar/devel/mujoco-ros2-control/models/franka_panda/mjx_scene.xml',
                'sim_dt': 0.002,
                'use_viewer': True,
                'control_mode': 'position'  
            }]
        ),
    ])

