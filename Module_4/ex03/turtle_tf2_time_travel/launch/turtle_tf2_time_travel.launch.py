from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim'
        ),
        ExecuteProcess(
            cmd=['ros2', 'service', 'call', '/spawn', 'turtlesim/srv/Spawn', "{x: 4.0, y: 2.0, theta: 0.0, name: 'turtle2'}"],
            output='screen'
        ),
        Node(
            package='turtle_tf2_time_travel',
            executable='turtle_tf2_broadcaster',
            name='broadcaster1',
            parameters=[
                {'turtlename': 'turtle1'}
            ]
        ),
        Node(
            package='turtle_tf2_time_travel',
            executable='turtle_tf2_broadcaster',
            name='broadcaster2',
            parameters=[
                {'turtlename': 'turtle2'}
            ]
        ),
        Node(
            package='turtle_tf2_time_travel',
            executable='turtle_tf2_listener',
            name='listener',
            parameters=[
                {'delay': 5.0}
            ]
        ),
    ])
