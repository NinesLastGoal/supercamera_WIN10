"""
SuperCamera ROS2 Launch File

Usage:
    ros2 launch supercamera_ros supercamera.launch.py
    ros2 launch supercamera_ros supercamera.launch.py topic:=/my/camera/image
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'device',
            default_value='/dev/supercamera',
            description='Path to the camera device'
        ),
        DeclareLaunchArgument(
            'topic',
            default_value='/supercamera/image_raw',
            description='Topic to publish images to'
        ),
        DeclareLaunchArgument(
            'frame_id',
            default_value='supercamera_link',
            description='TF frame ID for the camera'
        ),
        
        Node(
            package='supercamera_ros',
            executable='publisher',
            name='supercamera_publisher',
            parameters=[{
                'device': LaunchConfiguration('device'),
                'topic': LaunchConfiguration('topic'),
                'frame_id': LaunchConfiguration('frame_id'),
            }],
            output='screen'
        ),
    ])
