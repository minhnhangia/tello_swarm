"""
Launch file for Waypoint Server.

Starts the centralized waypoint reservation server for swarm coordination.
Drones can reserve waypoints to prevent collisions during multi-drone missions.

Usage:
    ros2 launch tello_swarm waypoint_server_launch.py
    ros2 launch tello_swarm waypoint_server_launch.py max_waypoints:=50
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for waypoint server."""
    
    # Declare launch arguments
    max_waypoints_arg = DeclareLaunchArgument(
        'max_waypoints',
        default_value='100',
        description='Maximum number of waypoints to support (0-N)'
    )
    
    expiration_duration_arg = DeclareLaunchArgument(
        'expiration_duration_s',
        default_value='8.0',
        description='Seconds before reservation expires without heartbeat'
    )
    
    # Create waypoint server node
    waypoint_server_node = Node(
        package='tello_swarm',
        executable='waypoint_server',
        name='waypoint_server',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'max_waypoints': LaunchConfiguration('max_waypoints'),
            'expiration_duration_s': LaunchConfiguration('expiration_duration_s'),
        }],
    )
    
    return LaunchDescription([
        max_waypoints_arg,
        expiration_duration_arg,
        waypoint_server_node,
    ])
