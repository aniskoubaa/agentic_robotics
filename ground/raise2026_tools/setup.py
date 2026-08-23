# Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>

from setuptools import setup

package_name = 'raise2026_tools'

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
    description='RAISE 2026 LLM-callable ROS 2 tools.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'gripper_server       = raise2026_tools.gripper_server:main',
            'move_to_pose_server  = raise2026_tools.move_to_pose_server:main',
            'navigation_server    = raise2026_tools.navigation_server:main',
            'detector_server      = raise2026_tools.detector_server:main',
            'inspector_server     = raise2026_tools.inspector_server:main',
            'grasp_server         = raise2026_tools.grasp_server:main',
            # 'inspect_plant_server = raise2026_tools.inspect_plant_server:main',
        ],
    },
)
