"""
Launch minimo para ver la capturadora USB con el driver usb_cam.

Publica:
    /camera/thermal/image_raw/compressed
    /camera/thermal/image_raw
    /camera/thermal/camera_info

Uso:
    ros2 launch drone_bringup usb_camera.launch.py
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
            default_value='30.0',
            description='FPS solicitados al dispositivo.',
        ),
        DeclareLaunchArgument(
            'pixel_format',
            default_value='raw_mjpeg',
            description='Formato de usb_cam. Para evitar conversion interna de MJPG usar raw_mjpeg.',
        ),
        DeclareLaunchArgument(
            'io_method',
            default_value='mmap',
            description='Metodo de I/O V4L2: mmap, read o userptr.',
        ),
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            namespace='camera/thermal',
            output='screen',
            parameters=[{
                'video_device': LaunchConfiguration('device_path'),
                'image_width': ParameterValue(LaunchConfiguration('frame_width'), value_type=int),
                'image_height': ParameterValue(LaunchConfiguration('frame_height'), value_type=int),
                'framerate': ParameterValue(LaunchConfiguration('fps'), value_type=float),
                'pixel_format': LaunchConfiguration('pixel_format'),
                'io_method': LaunchConfiguration('io_method'),
                'camera_frame_id': 'camera_thermal_optical',
                'camera_name': 'camera_thermal',
                'camera_info_url': 'package://usb_cam/config/camera_info.yaml',
                'brightness': -1,
                'contrast': -1,
                'saturation': -1,
                'sharpness': -1,
                'gain': -1,
                'white_balance': -1,
                'exposure': -1,
                'focus': -1,
            }],
        ),
        Node(
            package='image_transport',
            executable='republish',
            name='thermal_image_republish',
            output='screen',
            arguments=['compressed', 'raw'],
            remappings=[
                ('in/compressed', '/camera/thermal/image_raw/compressed'),
                ('out', '/camera/thermal/image_raw'),
            ],
        ),
    ])
