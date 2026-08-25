from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')
    default_params = PathJoinSubstitution(
        [FindPackageShare('px4_interface'), 'config', 'sitl_hover.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='YAML file with offboard_hover_node parameters'),
        DeclareLaunchArgument(
            'hover_duration_s',
            default_value='10.0',
            description='Seconds to hover after position error is within position_tolerance_m'),
        DeclareLaunchArgument(
            'px4_ready_timeout_s',
            default_value='30.0',
            description='Seconds to wait for VehicleStatus before aborting'),
        DeclareLaunchArgument(
            'prestream_duration_s',
            default_value='1.0',
            description='Minimum setpoint/heartbeat stream before OFFBOARD (s)'),
        DeclareLaunchArgument(
            'prestream_timeout_s',
            default_value='10.0',
            description='Max time allowed in PRESTREAM_SETPOINTS (s)'),
        DeclareLaunchArgument(
            'offboard_timeout_s',
            default_value='10.0',
            description='Max time to wait for nav_state OFFBOARD (s)'),
        DeclareLaunchArgument(
            'arm_timeout_s',
            default_value='10.0',
            description='Max time to wait for ARMING_STATE_ARMED (s)'),
        DeclareLaunchArgument(
            'landing_timeout_s',
            default_value='30.0',
            description='Max time to wait for land/disarm (s)'),
        DeclareLaunchArgument(
            'use_relative_target',
            default_value='true',
            description='If true, hover NED XY from live home odometry; Z = home_z - takeoff_height_m'),
        DeclareLaunchArgument(
            'takeoff_height_m',
            default_value='1.0',
            description='Relative takeoff height in metres (NED up = minus Z); must be > 0'),
        DeclareLaunchArgument(
            'relative_offset_x_m',
            default_value='0.0',
            description='Relative-mode NED North offset from home (m); unused if use_relative_target=false'),
        DeclareLaunchArgument(
            'relative_offset_y_m',
            default_value='0.0',
            description='Relative-mode NED East offset from home (m); unused if use_relative_target=false'),
        DeclareLaunchArgument(
            'position_tolerance_m',
            default_value='0.15',
            description='3D position error (m) that starts hover_duration_s; must be > 0'),
        DeclareLaunchArgument(
            'target_reach_timeout_s',
            default_value='45.0',
            description='Max seconds in HOVER to reach setpoint before land/abort; must be > 0'),
        DeclareLaunchArgument(
            'target_x', default_value='0.0', description='Absolute NED hover x (m) if use_relative_target=false'),
        DeclareLaunchArgument(
            'target_y', default_value='0.0', description='NED hover y (m)'),
        DeclareLaunchArgument(
            'target_z', default_value='-2.0', description='NED hover z (m, down positive)'),
        DeclareLaunchArgument(
            'target_yaw', default_value='0.0', description='NED yaw (rad)'),
        Node(
            package='px4_interface',
            executable='offboard_hover_node',
            name='offboard_hover_node',
            output='screen',
            parameters=[
                params_file,
                {
                    'hover_duration_s': LaunchConfiguration('hover_duration_s'),
                    'px4_ready_timeout_s': LaunchConfiguration('px4_ready_timeout_s'),
                    'prestream_duration_s': LaunchConfiguration('prestream_duration_s'),
                    'prestream_timeout_s': LaunchConfiguration('prestream_timeout_s'),
                    'offboard_timeout_s': LaunchConfiguration('offboard_timeout_s'),
                    'arm_timeout_s': LaunchConfiguration('arm_timeout_s'),
                    'landing_timeout_s': LaunchConfiguration('landing_timeout_s'),
                    'use_relative_target': LaunchConfiguration('use_relative_target'),
                    'takeoff_height_m': LaunchConfiguration('takeoff_height_m'),
                    'relative_offset_x_m': LaunchConfiguration('relative_offset_x_m'),
                    'relative_offset_y_m': LaunchConfiguration('relative_offset_y_m'),
                    'position_tolerance_m': LaunchConfiguration('position_tolerance_m'),
                    'target_reach_timeout_s': LaunchConfiguration('target_reach_timeout_s'),
                    'target_x': LaunchConfiguration('target_x'),
                    'target_y': LaunchConfiguration('target_y'),
                    'target_z': LaunchConfiguration('target_z'),
                    'target_yaw': LaunchConfiguration('target_yaw'),
                },
            ],
        ),
    ])
