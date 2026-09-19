# Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>

from setuptools import setup

package_name = 'agr_tb_examples'

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
    description='TurtleBot elementary examples: one mobile-robot idea per script.',
    license='MIT',
    entry_points={
        'console_scripts': [
            '01_read_odometry  = agr_tb_examples.e01_read_odometry:main',
            '02_drive          = agr_tb_examples.e02_drive:main',
            '03_read_lidar     = agr_tb_examples.e03_read_lidar:main',
            '04_get_image      = agr_tb_examples.e04_get_image:main',
            '05_drive_a_square = agr_tb_examples.e05_drive_a_square:main',
            '06_avoid_obstacle = agr_tb_examples.e06_avoid_obstacle:main',
        ],
    },
)
