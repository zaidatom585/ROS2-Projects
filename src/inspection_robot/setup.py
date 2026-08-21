from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'inspection_robot'

setup(
    name=package_name,
    version='0.0.1',

    packages=find_packages(exclude=['test']),

    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),

        (
            'share/' + package_name,
            ['package.xml']
        ),

        # Launch files
        (
            os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))
        ),

        # URDF / Xacro files
        (
            os.path.join('share', package_name, 'urdf'),
            glob(os.path.join('urdf', '*'))
        ),

        # RViz configuration
        (
            os.path.join('share', package_name, 'rviz'),
            glob(os.path.join('rviz', '*'))
        ),

        # Meshes
        (
            os.path.join('share', package_name, 'meshes'),
            glob(os.path.join('meshes', '*'))
        ),

        # Gazebo worlds
        (
            os.path.join('share', package_name, 'worlds'),
            glob(os.path.join('worlds', '*'))
        ),
    ],

    install_requires=['setuptools'],

    zip_safe=True,

    maintainer='zaidatom585',
    maintainer_email='zaidatom585@todo.todo',

    description='Inspection robot package',

    license='MIT',

    tests_require=['pytest'],

    entry_points={
        'console_scripts': [
            'drive = inspection_robot.drive_node:main',
            'lidar = inspection_robot.lidar_node:main',
            'avoid = inspection_robot.obstacle_avoidance:main',
            'camera = inspection_robot.camera_node:main',
            'maze_navigator = maze_navigator.maze_navigator:main',
        ],
    },
)