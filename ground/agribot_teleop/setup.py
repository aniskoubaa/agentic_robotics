# Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>

from setuptools import setup
from glob import glob

package_name = 'agribot_teleop'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/templates', glob('templates/*.html')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Anis Koubaa',
    maintainer_email='anis.koubaa@gmail.com',
    description='AgriBot teleop: keyboard, joystick, phone.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'teleop_keyboard = agribot_teleop.teleop_keyboard:main',
            'teleop_phone    = agribot_teleop.teleop_phone:main',
            'camera_view     = agribot_teleop.camera_view:main',
        ],
    },
)
