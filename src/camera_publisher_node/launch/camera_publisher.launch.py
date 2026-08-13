"""
Launch del nodo de video en vivo desde la capturadora USB.

Uso:
    ros2 launch camera_publisher_node camera_publisher.launch.py

Con ruta estable:
    ros2 launch camera_publisher_node camera_publisher.launch.py device_path:=/dev/v4l/by-id/usb-UGREEN_25854-video-index0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'device_path',
            default_value='/dev/video3',
            description='Ruta V4L2 de la capturadora USB.',
        ),
        DeclareLaunchArgument(
            'device_index',
            default_value='3',
            description='Indice de video si device_path esta vacio.',
        ),
        DeclareLaunchArgument(
            'frame_width',
            default_value='1920',
            description='Ancho de captura solicitado.',
        ),
        DeclareLaunchArgument(
            'frame_height',
            default_value='1080',
            description='Alto de captura solicitado.',
        ),
        DeclareLaunchArgument(
            'fps',
            default_value='30',
            description='FPS solicitados al dispositivo.',
        ),
        DeclareLaunchArgument(
            'publish_rate',
            default_value='30',
            description='Frecuencia de publicacion ROS.',
        ),
        DeclareLaunchArgument(
            'pixel_format',
            default_value='MJPG',
            description='FourCC solicitado al dispositivo.',
        ),
        DeclareLaunchArgument(
            'use_compression',
            default_value='true',
            description='Publicar tambien /camera/thermal/image_compressed.',
        ),
        Node(
            package='camera_publisher_node',
            executable='camera_publisher',
            name='camera_publisher',
            output='screen',
            parameters=[{
                'device_path': LaunchConfiguration('device_path'),
                'device_index': ParameterValue(LaunchConfiguration('device_index'), value_type=int),
                'frame_width': ParameterValue(LaunchConfiguration('frame_width'), value_type=int),
                'frame_height': ParameterValue(LaunchConfiguration('frame_height'), value_type=int),
                'fps': ParameterValue(LaunchConfiguration('fps'), value_type=int),
                'publish_rate': ParameterValue(LaunchConfiguration('publish_rate'), value_type=int),
                'pixel_format': LaunchConfiguration('pixel_format'),
                'use_compression': ParameterValue(LaunchConfiguration('use_compression'), value_type=bool),
            }],
        ),
    ])
