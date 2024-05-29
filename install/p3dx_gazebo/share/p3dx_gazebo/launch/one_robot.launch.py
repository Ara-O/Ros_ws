import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
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

        OpaqueFunction(function=spawn_robot),
    ])

def spawn_robot(context, *args, **kwargs):
    # Get the initialization pose
    init_pose = LaunchConfiguration('init_pose').perform(context).split()
    
    # Extract x, y, z values
    x = init_pose[1]
    y = init_pose[3]
    z = init_pose[5]

    return [
        # Spawn the robot in Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='urdf_spawner',
            output='screen',
            arguments=['-entity', LaunchConfiguration('robot_name').perform(context), '-topic', 'robot_description',
                       '-x', x, '-y', y, '-z', z]
        ),

        # Joint State Publisher
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[{'use_gui': False}],
            remappings=[('/robot_description', 'robot_description')]
        ),
    ]

if __name__ == '__main__':
    generate_launch_description()
