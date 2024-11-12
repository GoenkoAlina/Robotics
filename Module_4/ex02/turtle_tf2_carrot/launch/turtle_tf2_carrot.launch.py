from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    radius_arg = DeclareLaunchArgument(
        'radius',
        default_value='2.0',
        description='Radius of the carrot frame'
    )

    direction_arg = DeclareLaunchArgument(
        'direction_of_rotation',
        default_value='1',
        description='Direction of rotation: 1 for clockwise, -1 for counterclockwise'
    )
    
    return LaunchDescription([
    	radius_arg,
        direction_arg,
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
            package='turtle_tf2_carrot',
            executable='turtle_tf2_broadcaster',
            name='broadcaster1',
            parameters=[
                {'turtlename': 'turtle1'}
            ]
        ),
        Node(
            package='turtle_tf2_carrot',
            executable='turtle_tf2_broadcaster',
            name='broadcaster2',
            parameters=[
                {'turtlename': 'turtle2'}
            ]
        ),
        Node(
            package='turtle_tf2_carrot',
            executable='carrot_tf2_broadcaster',
            name='carrot_broadcaster',
            parameters=[
                {'radius': LaunchConfiguration('radius')},
                {'direction_of_rotation': LaunchConfiguration('direction_of_rotation')}
            ]
        ),
        Node(
            package='turtle_tf2_carrot',
            executable='turtle_tf2_listener',
            name='listener',
        ),
    ])
