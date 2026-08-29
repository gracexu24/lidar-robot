from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'my_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py'),
        ),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lidar',
    maintainer_email='lidar@todo.todo',
    description='Four-wheel GPIO mobile robot with D500 LiDAR navigation',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'base_controller = my_robot.base_controller:main',
            'drive = my_robot.manual_drive:main',
            'lidar_reader = my_robot.lidar_reader:main',
            'mpu6050_driver = my_robot.mpu6050_driver:main',
        ],
    },
)
