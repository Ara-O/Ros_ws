import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Define package paths
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_p3dx_gazebo = get_package_share_directory('p3dx_gazebo')

    # Declare the launch arguments
    return LaunchDescription([
        DeclareLaunchArgument(
            'paused',
            default_value='false',
            description='Start the simulation paused'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock'
        ),
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Run with GUI'
        ),
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='Run headless (no GUI)'
        ),
        DeclareLaunchArgument(
            'debug',
            default_value='false',
            description='Run in debug mode'
        ),
        DeclareLaunchArgument(
            'world_name',
            default_value=os.path.join(pkg_p3dx_gazebo, 'worlds', 'feb2_scenario.world'),
            description='World file to load'
        ),

        # Include the Gazebo launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')),
            launch_arguments={
                'world': LaunchConfiguration('world_name'),
                'paused': LaunchConfiguration('paused'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'gui': LaunchConfiguration('gui'),
                'headless': LaunchConfiguration('headless'),
                'debug': LaunchConfiguration('debug')
            }.items(),
        ),

        # Include the robots launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_p3dx_gazebo, 'launch', 'robots44211.launch.py'))
        )
    ])
