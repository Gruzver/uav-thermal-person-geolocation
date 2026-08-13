"""
Launch para publicar el stream RTSP del drone como topico ROS2.

El nodo auto-descubre el stream activo en MediaMTX si no se pasa rtsp_url.

Uso:
    ros2 launch drone_bringup rtsp_camera.launch.py
    ros2 launch drone_bringup rtsp_camera.launch.py rtsp_url:=rtsp://127.0.0.1:8554/live/<stream-id>
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'rtsp_url',
            default_value='',
            description='URL RTSP completa. Si vacio, auto-descubre desde MediaMTX.',
        ),
        DeclareLaunchArgument(
            'mediamtx_api',
            default_value='http://127.0.0.1:9997',
            description='URL base de la API de MediaMTX para auto-descubrimiento.',
        ),
        DeclareLaunchArgument(
            'publish_rate',
            default_value='30',
            description='Frecuencia de publicacion en Hz.',
        ),
        Node(
            package='camera_publisher_node',
            executable='rtsp_publisher',
            name='rtsp_publisher',
            output='screen',
            parameters=[{
                'rtsp_url': LaunchConfiguration('rtsp_url'),
                'mediamtx_api': LaunchConfiguration('mediamtx_api'),
                'publish_rate': ParameterValue(
                    LaunchConfiguration('publish_rate'), value_type=int
                ),
                'use_compression': True,
            }],
        ),
    ])
