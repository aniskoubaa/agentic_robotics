# Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>

from setuptools import setup

package_name = 'agr_arm_tools'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Anis Koubaa',
    maintainer_email='anis.koubaa@gmail.com',
    description='Kinematics, motion helper and service servers for the bench UR5e.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'gripper_server      = agr_arm_tools.gripper_server:main',
            'move_to_pose_server = agr_arm_tools.move_to_pose_server:main',
            'grasp_server        = agr_arm_tools.grasp_server:main',
            'workspace           = agr_arm_tools.workspace:main',
        ],
    },
)
