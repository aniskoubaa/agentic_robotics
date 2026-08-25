# Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>

from setuptools import setup

package_name = 'raisebot_tools'

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
    description='RaiseBot LLM-callable ROS 2 tools.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'gripper_server       = raisebot_tools.gripper_server:main',
            'move_to_pose_server  = raisebot_tools.move_to_pose_server:main',
            'navigation_server    = raisebot_tools.navigation_server:main',
            'detector_server      = raisebot_tools.detector_server:main',
            'inspector_server     = raisebot_tools.inspector_server:main',
            'grasp_server         = raisebot_tools.grasp_server:main',
            # 'inspect_plant_server = raisebot_tools.inspect_plant_server:main',
        ],
    },
)
