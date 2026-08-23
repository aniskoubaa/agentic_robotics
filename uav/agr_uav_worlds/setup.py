# Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>
from setuptools import setup
from glob import glob

package_name = 'agr_uav_worlds'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/worlds', glob('worlds/*.sdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Anis Koubaa',
    maintainer_email='anis.koubaa@gmail.com',
    description='Agentic Robotics — Gazebo worlds for UAV labs.',
    license='MIT',
    entry_points={'console_scripts': []},
)
