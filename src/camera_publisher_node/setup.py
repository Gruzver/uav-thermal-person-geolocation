from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'camera_publisher_node'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Gruzver Romario Phocco Caceres',
    maintainer_email='gruzver.phocco@pucp.edu.pe',
    description='Fuentes de video en vivo: capturadora USB (HDMI) y stream RTSP via MediaMTX.',
    license='MIT',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'camera_publisher = camera_publisher_node.camera_publisher:main',
            'rtsp_publisher = camera_publisher_node.rtsp_publisher:main',
        ],
    },
)
