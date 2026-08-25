# Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>

from setuptools import setup

package_name = 'agr_legged_examples'

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
    description='Go2 elementary examples: one legged-robotics concept per script.',
    license='MIT',
    entry_points={
        'console_scripts': [
            '01_read_joints   = agr_legged_examples.e01_read_joints:main',
            '02_read_imu      = agr_legged_examples.e02_read_imu:main',
            '03_get_image     = agr_legged_examples.e03_get_image:main',
            '04_change_pose   = agr_legged_examples.e04_change_pose:main',
            '05_walk          = agr_legged_examples.e05_walk:main',
            '06_walk_a_square = agr_legged_examples.e06_walk_a_square:main',
        ],
    },
)
