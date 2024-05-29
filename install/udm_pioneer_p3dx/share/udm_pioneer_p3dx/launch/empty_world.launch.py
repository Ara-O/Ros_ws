import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument('paused', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('extra_gazebo_args', default_value=''),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('debug', default_value='false'),
        DeclareLaunchArgument('physics', default_value='ode'),
        DeclareLaunchArgument('verbose', default_value='false'),
        DeclareLaunchArgument('world_name', default_value='worlds/empty.world'),

        # Set use_sim_time parameter
        Node(
            package='ros2 param package',
            executable='parameter_node',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            condition=IfCondition(LaunchConfiguration('use_sim_time'))
        ),

        # Set command arguments
        Node(
            package='gazebo_ros',
            executable=LaunchConfiguration('script_type'),
            name='gazebo',
            output='screen',
            arguments=[
                LaunchConfiguration('command_arg1'),
                LaunchConfiguration('command_arg2'),
                LaunchConfiguration('command_arg3'),
                '-e', LaunchConfiguration('physics'),
                LaunchConfiguration('extra_gazebo_args'),
                LaunchConfiguration('world_name')
            ]
        ),

        # Start gazebo client
        GroupAction([
            Node(
                package='gazebo_ros',
                executable='gzclient',
                name='gazebo_gui',
                output='screen',
                condition=IfCondition(LaunchConfiguration('gui'))
            )
        ]),
    ])
