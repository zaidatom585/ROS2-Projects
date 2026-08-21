from setuptools import find_packages, setup

package_name = 'maze_navigator'

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
    ],

    install_requires=['setuptools'],

    zip_safe=True,

    maintainer='zaidatom585',
    maintainer_email='zaidatom585@gmail.com',

    description='Autonomous LiDAR maze navigator',

    license='MIT',

    tests_require=['pytest'],

    entry_points={
        'console_scripts': [
            'maze_navigator = maze_navigator.maze_navigator:main',
        ],
    },
)