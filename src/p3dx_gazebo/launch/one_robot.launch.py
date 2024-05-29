import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Define package paths
    pkg_p3dx_description = get_package_share_directory('p3dx_description')
    pkg_xacro = get_package_share_directory('xacro')

    # Declare launch arguments
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_name',
            default_value='robot',
            description='Name of the robot'
        ),
        DeclareLaunchArgument(
            'init_pose',
            default_value='-x 0 -y 0 -z 0',
            description='Initial pose of the robot'
        ),
        DeclareLaunchArgument(
            'model',
            default_value=os.path.join(pkg_p3dx_description, 'urdf', 'pioneer3dx.xacro'),
            description='Path to the robot URDF file'
        ),
        
        # Load the URDF into the parameter server
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command(['xacro ', LaunchConfiguration('model')])
            }]
        ),
        
        # Spawn the robot in Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='urdf_spawner',
            output='screen',
        #   arguments=['-entity', LaunchConfiguration('robot_name'), '-topic', 'robot_description', LaunchConfiguration('init_pose')]
            arguments=['-entity', LaunchConfiguration('robot_name'), '-topic', 'robot_description',
                       '-x', init_pose[1], '-y', init_pose[3], '-z', init_pose[5]]
        ),
        
        # Joint State Publisher
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[{'use_gui': False}],
            remappings=[('/robot_description', 'robot_description')]
        ),
        
        # Robot State Publisher
        #Node(
        #     package='robot_state_publisher',
        #     executable='robot_state_publisher',
        #     name='robot_state_publisher',
        #     output='screen'
        # ),
    ])
