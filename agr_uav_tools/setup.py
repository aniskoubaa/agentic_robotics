# Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>
from setuptools import setup

package_name = 'agr_uav_tools'

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
    description='Agentic Robotics — UAV runtime nodes and PX4 topic helpers.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'vehicle_monitor = agr_uav_tools.vehicle_monitor:main',
            'list_airframes  = agr_uav_tools.list_airframes:main',
        ],
    },
)
