# Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>

from setuptools import setup

package_name = 'raisebot_demos'

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
    description='RaiseBot demos and diagnostics.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'diagnose         = raisebot_demos.diagnose:main',
            'demo_greenhouse  = raisebot_demos.demo_greenhouse:main',
        ],
    },
)
