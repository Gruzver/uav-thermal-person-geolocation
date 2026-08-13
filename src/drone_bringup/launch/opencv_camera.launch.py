"""
Launch minimo para ver la capturadora USB usando OpenCV/V4L2.

Publica:
    /camera/thermal/image_raw

Uso:
    ros2 launch drone_bringup opencv_camera.launch.py
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
                'use_compression': False,
            }],
        ),
    ])
