from setuptools import find_packages, setup

package_name = 'map_server_node'

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
    description='Agregacion de posiciones georreferenciadas y exportacion de resultados.',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'map_server = map_server_node.map_server:main',
        ],
    },
)
