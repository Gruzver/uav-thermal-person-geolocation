from setuptools import find_packages, setup

package_name = 'georeferencing_node'

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
    description='Conversion de detecciones en pixeles a coordenadas GPS, con filtrado de Kalman.',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'georeferencer = georeferencing_node.georeferencer:main',
            'georeferencer_nrt = georeferencing_node.georeferencer_nrt:main',
        ],
    },
)
