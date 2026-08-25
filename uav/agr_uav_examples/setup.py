# Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>

from setuptools import setup

package_name = 'agr_uav_examples'

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
    description='UAV elementary examples: one PX4/ROS 2 concept per script.',
    license='MIT',
    entry_points={
        'console_scripts': [
            '01_read_telemetry  = agr_uav_examples.e01_read_telemetry:main',
            '02_arm_and_disarm  = agr_uav_examples.e02_arm_and_disarm:main',
            '03_takeoff_and_land = agr_uav_examples.e03_takeoff_and_land:main',
            '04_fly_a_square    = agr_uav_examples.e04_fly_a_square:main',
            '05_get_image       = agr_uav_examples.e05_get_image:main',
            '06_list_airframes  = agr_uav_examples.e06_list_airframes:main',
        ],
    },
)
