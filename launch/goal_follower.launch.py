from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')
    default_params = PathJoinSubstitution(
        [FindPackageShare('px4_interface'), 'config', 'goal_follower.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='YAML file with goal_follower_node parameters'),
        Node(
            package='px4_interface',
            executable='goal_follower_node',
            name='goal_follower_node',
            output='screen',
            parameters=[params_file],
        ),
    ])
