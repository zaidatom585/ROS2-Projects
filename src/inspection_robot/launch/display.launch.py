import os
from glob import glob

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # --- Robot description ---
    xacro_file = PathJoinSubstitution([
        FindPackageShare('inspection_robot'),
        'urdf',
        'inspection_robot.urdf.xacro'
    ])

    robot_description = {
        'robot_description': Command([
            FindExecutable(name='xacro'),
            ' ',
            xacro_file
        ])
    }

    # --- RViz config (FIXED: use 'rviz' folder to match your package structure) ---
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('inspection_robot'),
        'rviz',           # <-- Changed from 'config' to 'rviz'
        'display.rviz'    # <-- Use whatever your file is actually named
    ])

    return LaunchDescription([

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[robot_description]
        ),

        # ADD THIS: Publishes dummy joint states so TF frames work
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen'
        ),

        # OPTIONAL: Use this INSTEAD if you want sliders to move joints manually
        # Node(
        #     package='joint_state_publisher_gui',
        #     executable='joint_state_publisher_gui',
        #     name='joint_state_publisher_gui',
        #     output='screen'
        # ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file]
        ),

    ])