from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'rover_nav'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml') + glob('config/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tarun',
    maintainer_email='tarun@todo.todo',
    description='Indoor waypoint navigation for Traxxas rover',
    license='MIT',
    entry_points={
        'console_scripts': [
            'zed_mavros_bridge = rover_nav.zed_mavros_bridge:main',
            'arm_and_go = rover_nav.arm_and_go:main',
            'cmd_vel_stamper = rover_nav.cmd_vel_stamper:main',
        ],
    },
)
