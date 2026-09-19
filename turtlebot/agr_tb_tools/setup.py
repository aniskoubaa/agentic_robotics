# Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>

from glob import glob

from setuptools import setup

package_name = 'agr_tb_tools'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Anis Koubaa',
    maintainer_email='anis.koubaa@gmail.com',
    description='Robot registry and a Base motion helper for the official TurtleBot simulators.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'list_robots = agr_tb_tools.registry:main',
        ],
    },
)
