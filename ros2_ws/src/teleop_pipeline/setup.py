from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'teleop_pipeline'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ahmad Gazar',
    maintainer_email='ahmad.gazar@neura-robotcs.com',
    description='Teleop -> MINK IK -> Impedance -> MuJoCo torque pipeline',
    license='MIT',
    entry_points={
        'console_scripts': [
            'pose_target_pub = teleop_pipeline.pose_target_pub:main',
            'mink_ik_node   = teleop_pipeline.mink_ik_node:main',
            'impedance_node = teleop_pipeline.impedance_node:main',
            'torque_router  = teleop_pipeline.torque_router:main',
            'mujoco_sim_bridge = teleop_pipeline.mujoco_sim_bridge:main',
            'mujoco_visualizer = teleop_pipeline.mujoco_visualizer:main',
        ],
    },
)
