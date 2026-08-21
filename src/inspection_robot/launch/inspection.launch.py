from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    package_dir = get_package_share_directory('inspection_robot')

    urdf_file = os.path.join(
        package_dir,
        'urdf',
        'inspection_robot.urdf.xacro'
    )

    world_file = os.path.join(
        package_dir,
        'worlds',
        'obstacle_world.world'
    )

    rviz_file = os.path.join(
        package_dir,
        'rviz',
        'display.rviz'
    )

    gazebo_launch = os.path.join(
        get_package_share_directory('gazebo_ros'),
        'launch',
        'gazebo.launch.py'
    )

    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )

    return LaunchDescription([

        # =========================================
        # GAZEBO
        # =========================================

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
            launch_arguments={
                'world': world_file
            }.items()
        ),

        # =========================================
        # ROBOT STATE PUBLISHER
        # =========================================

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[
                {
                    'robot_description': robot_description
                }
            ],
            output='screen'
        ),

        # =========================================
        # SPAWN ROBOT INTO GAZEBO
        # =========================================

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-topic',
                'robot_description',
                '-entity',
                'inspection_robot',
                '-x',
                '0.0',
                '-y',
                '0.0',
                '-z',
                '0.20'
            ],
            output='screen'
        ),

        # =========================================
        # RVIZ
        # =========================================

        Node(
            package='rviz2',
            executable='rviz2',
            arguments=[
                '-d',
                rviz_file
            ],
            output='screen'
        )

    ])