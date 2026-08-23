# Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>

from setuptools import setup

package_name = 'raise2026_demos'

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
    description='RAISE 2026 lecture demos.',
    license='MIT',
    entry_points={
        'console_scripts': [
            # 'd1l1_tools_demo = raise2026_demos.d1l1_tools_demo:main',
            # 'd1l2_agentic_inspector = raise2026_demos.d1l2_agentic_inspector:main',
            # 'd2l1_teleop_record = raise2026_demos.d2l1_teleop_record:main',
            # 'd2l2_vla_rollout = raise2026_demos.d2l2_vla_rollout:main',
            # 'd3l1_planner_executor = raise2026_demos.d3l1_planner_executor:main',
        ],
    },
)
