import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Define package path
    pkg_p3dx_gazebo = get_package_share_directory('p3dx_gazebo')

    # Declare launch arguments
    return LaunchDescription([
	DeclareLaunchArgument(
	    'robot1',
	    default_value='true',
	    description='Flag to include robot1'
	),
	DeclareLaunchArgument(
	    'robot2',
	    default_value='true',
	    description='Flag to include robot2'
	),
        DeclareLaunchArgument(
            'init_pose_robot1',
            default_value='-x -1.945 -y -0.50751 -z 0.0',
            description='Initial pose for robot1'
        ),
        DeclareLaunchArgument(
            'robot_name_robot1',
            default_value='robot1',
            description='Robot name for robot1'
        ),
        DeclareLaunchArgument(
            'init_pose_robot2',
            default_value='-x 1.7558 -y 1.6492 -z 0.0',
            description='Initial pose for robot2'
        ),
        DeclareLaunchArgument(
            'robot_name_robot2',
            default_value='robot2',
            description='Robot name for robot2'
        ),

        # Group for robot1
        GroupAction([
            DeclareLaunchArgument(
                'tf_prefix_robot1',
                default_value='robot1_tf',
                description='TF prefix for robot1'
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg_p3dx_gazebo, 'launch', 'one_robot.launch.py')),
                launch_arguments={
                    'init_pose': LaunchConfiguration('init_pose_robot1'),
                    'robot_name': LaunchConfiguration('robot_name_robot1'),
                    'tf_prefix': LaunchConfiguration('tf_prefix_robot1')
                }.items()
            ),
        ], condition=IfCondition(LaunchConfiguration('robot1'))),

        # Group for robot2
        GroupAction([
            DeclareLaunchArgument(
                'tf_prefix_robot2',
                default_value='robot2_tf',
                description='TF prefix for robot2'
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg_p3dx_gazebo, 'launch', 'one_robot.launch.py')),
                launch_arguments={
                    'init_pose': LaunchConfiguration('init_pose_robot2'),
                    'robot_name': LaunchConfiguration('robot_name_robot2'),
                    'tf_prefix': LaunchConfiguration('tf_prefix_robot2')
                }.items()
            ),
        ], condition=IfCondition(LaunchConfiguration('robot2'))),
    ])
