# Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>

from setuptools import setup
from glob import glob

package_name = 'raise2026_teleop'

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
    description='RAISE 2026 teleop: keyboard, joystick, phone.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'teleop_keyboard = raise2026_teleop.teleop_keyboard:main',
            'teleop_phone    = raise2026_teleop.teleop_phone:main',
            'camera_view     = raise2026_teleop.camera_view:main',
        ],
    },
)
