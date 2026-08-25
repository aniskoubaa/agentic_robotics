# Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>

from setuptools import setup

package_name = 'agr_uav_teleop'

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
    description='UAV teleop: keyboard flight in PX4 offboard mode, camera view.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'teleop_keyboard = agr_uav_teleop.teleop_keyboard:main',
            'camera_view     = agr_uav_teleop.camera_view:main',
        ],
    },
)
