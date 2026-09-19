# Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>

from glob import glob

from setuptools import setup

package_name = 'agr_arm_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Anis Koubaa',
    maintainer_email='anis.koubaa@gmail.com',
    description='Launch files and topic bridge for the bench-mounted UR5e arm.',
    license='MIT',
    entry_points={'console_scripts': []},
)
