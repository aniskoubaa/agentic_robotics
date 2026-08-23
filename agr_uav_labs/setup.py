# Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>
from setuptools import setup

package_name = 'agr_uav_labs'

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
    description='Agentic Robotics — numbered student exercises for the UAV track.',
    license='MIT',
    entry_points={
        'console_scripts': [
            '01_check_bridge = agr_uav_labs.lab01_check_bridge:main',
        ],
    },
)
