# Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>

from setuptools import setup
from glob import glob

package_name = 'raise2026_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py') + glob('launch/*.xml')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Anis Koubaa',
    maintainer_email='anis.koubaa@gmail.com',
    description='RAISE 2026 bringup launch files.',
    license='MIT',
    entry_points={'console_scripts': []},
)
