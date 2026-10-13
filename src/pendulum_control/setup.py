import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'pendulum_control'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # launch/ 目录也要装进 install/，否则 ros2 launch 找不到
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='griffin-BC',
    maintainer_email='griffin-BC@users.noreply.github.com',
    description='Cart-pole 倒立摆的 PD / LQR 最优控制闭环（ROS 2）',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'plant = pendulum_control.plant_node:main',
            'pd    = pendulum_control.pd_controller:main',
            'lqr   = pendulum_control.lqr_controller:main',
            'viz   = pendulum_control.visualizer:main',
        ],
    },
)
