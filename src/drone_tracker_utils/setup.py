from setuptools import find_packages, setup

package_name = 'drone_tracker_utils'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Gruzver Romario Phocco Caceres',
    maintainer_email='gruzver.phocco@pucp.edu.pe',
    description='Utilidades compartidas: proyeccion pixel-GPS, parser de telemetria SRT, filtro de Kalman y visualizador de FOV.',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
