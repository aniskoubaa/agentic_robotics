# Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>

from setuptools import setup

package_name = 'agr_arm_examples'

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
    description='Bench UR5e elementary examples: one manipulation idea per script.',
    license='MIT',
    entry_points={
        'console_scripts': [
            '01_read_joints    = agr_arm_examples.e01_read_joints:main',
            '02_move_a_joint   = agr_arm_examples.e02_move_a_joint:main',
            '03_get_image      = agr_arm_examples.e03_get_image:main',
            '04_move_to_pose   = agr_arm_examples.e04_move_to_pose:main',
            '05_move_to_xyz    = agr_arm_examples.e05_move_to_xyz:main',
            '06_pick_and_place = agr_arm_examples.e06_pick_and_place:main',
        ],
    },
)
