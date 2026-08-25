# Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>

from setuptools import setup

package_name = 'raisebot_examples'

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
    description='RaiseBot elementary examples: one ROS 2 concept per script.',
    license='MIT',
    entry_points={
        'console_scripts': [
            '01_drive           = raisebot_examples.e01_drive:main',
            '02_read_lidar      = raisebot_examples.e02_read_lidar:main',
            '03_get_image       = raisebot_examples.e03_get_image:main',
            '04_aim_the_ptz     = raisebot_examples.e04_aim_the_ptz:main',
            '05_call_a_service  = raisebot_examples.e05_call_a_service:main',
            '06_navigate        = raisebot_examples.e06_navigate:main',
        ],
    },
)
