from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    panda_joints = [
        "panda_joint1","panda_joint2","panda_joint3","panda_joint4",
        "panda_joint5","panda_joint6","panda_joint7"
    ]
    return LaunchDescription([
        Node(package='teleop_pipeline', executable='pose_target_pub', name='pose_target_pub',
             parameters=[{'position':[0.5,0.0,0.4],'orientation_xyzw':[0,0,0,1]}]),

        Node(package='teleop_pipeline', executable='mink_ik_node', name='mink_ik',
             parameters=[{
                'use_robot_descriptions': True,
                'robot_description_pkg': 'panda_mj_description',
                'variant': '',  # set to "panda_nohand" if you want no gripper
                'end_effector_site': 'attachment_site',  # try 'panda_hand_tcp' if you use a hand variant
                'joint_names': panda_joints,
                'ik_dt': 0.005
             }]),

        Node(package='teleop_pipeline', executable='impedance_node', name='impedance',
             parameters=[{'joint_names': panda_joints,
                          'k_diag':[80,80,60,40,30,20,15],
                          'd_diag':[4,4,3,2,2,1.5,1]}]),

        Node(package='teleop_pipeline', executable='torque_router', name='router'),

        # MuJoCo sim node (menagerie panda XML is loaded inside mink_ik; for the bridge, pass a path too)
        Node(package='mujoco_ros2_control', executable='mujoco_sim_node', name='mujoco_sim',
             output='screen',
             parameters=[{
                # If your bridge needs an explicit path, reuse robot_descriptions here too:
                # Set a dummy path (not used if your bridge loads internally); else hard-code panda_mj_description.MJCF_PATH
                'model_path': '/root/.cache/robot_descriptions/panda_mj_description/*.xml',
                'publish_joint_states_topic': '/mujoco/joint_states',
                'effort_command_topic': '/mujoco/effort_command',
                'control_mode': 'TORQUE'
             }]),
    ])
