from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = LaunchConfiguration('config_file')
    read_only = LaunchConfiguration('read_only')

    default_config = PathJoinSubstitution([
        FindPackageShare('golfcart_low_level'),
        'config',
        'golfcart_low_level.yaml',
    ])

    # Overrides the read_only value in the YAML for both nodes at once, so a
    # telemetry-only run needs no file edit. The launch argument arrives as a
    # string, so it must be cast or the nodes reject it as a type mismatch.
    read_only_override = {
        'read_only': ParameterValue(read_only, value_type=bool),
    }

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config,
            description='YAML file with golfcart_low_level node parameters.',
        ),
        DeclareLaunchArgument(
            'read_only',
            default_value='false',
            description='true = read vehicle telemetry only; both nodes stop '
                        'transmitting on CAN, so speed, brake and steering are '
                        'never commanded.',
        ),
        Node(
            package='golfcart_low_level',
            executable='ros_can_bridge',
            name='ros_can_bridge',
            output='screen',
            parameters=[config_file, read_only_override],
        ),
        Node(
            package='golfcart_low_level',
            executable='steering_node',
            name='steering_node',
            output='screen',
            parameters=[config_file, read_only_override],
        ),
    ])
