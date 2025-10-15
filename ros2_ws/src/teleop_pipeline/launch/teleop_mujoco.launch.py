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
            parameters=[{
                'position': [0.5, 0.0, 0.4],
                'orientation_xyzw': [0.0, 0.0, 0.0, 1.0]
            }]
        ),
        
        Node(
            package='teleop_pipeline',
            executable='mink_ik_node',
            name='mink_ik',
            parameters=[{
                'use_robot_descriptions': True,
                'robot_description_pkg': 'panda_mj_description',
                'end_effector_frame': 'hand',
                'end_effector_frame_type': 'body',
                'joint_names': panda_joints,
                'ik_dt': 0.005
            }]
        ),
        
        Node(
            package='teleop_pipeline',
            executable='impedance_node',
            name='impedance',
            parameters=[{
                'joint_names': panda_joints,
                'k_diag': [80.0, 80.0, 60.0, 40.0, 30.0, 20.0, 15.0],
                'd_diag': [4.0, 4.0, 3.0, 2.0, 2.0, 1.5, 1.0]
            }]
        ),
        
        Node(
            package='teleop_pipeline',
            executable='torque_router',
            name='router'
        ),
        
        Node(
            package='teleop_pipeline',
            executable='mujoco_sim_bridge',
            name='mujoco_sim',
            output='screen',
            parameters=[{
                'use_robot_descriptions': True,
                'robot_description_pkg': 'panda_mj_description',
                'sim_dt': 0.002,
                'use_viewer': False
            }]
        ),
    ])
