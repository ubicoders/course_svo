from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    sys_id_arg = DeclareLaunchArgument(
        'sys_id',
        default_value='0',
        description='System ID for the robot (default: 0)'
    )

    bridge_node = Node(
        package='vrobots_ros2_bridge',
        executable='bridge_node',
        name='bridge_node',
        parameters=[{'sys_id': LaunchConfiguration('sys_id')}],
        output='screen'
    )

    cam_bridge_node = Node(
        package='vrobots_ros2_bridge',
        executable='cam_bridge_node',
        name='cam_bridge_node',
        parameters=[{'sys_id': LaunchConfiguration('sys_id')}],
        output='screen'
    )

    cmd_bridge_node = Node(
        package='vrobots_ros2_bridge',
        executable='cmd_bridge_node',
        name='cmd_bridge_node',
        parameters=[{'sys_id': LaunchConfiguration('sys_id')}],
        output='screen'
    )

    return LaunchDescription([
        sys_id_arg,
        bridge_node,
        cam_bridge_node,
        cmd_bridge_node
    ])
