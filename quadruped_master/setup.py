from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'quadruped_master'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=['quadruped_master', 'quadruped_master.*'], exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='erick',
    maintainer_email='erick@todo.todo',
    description='Paquete para control del cuadrúpedo en ROS 2.',
    license='Apache License 2.0',
    extras_require={
        'test': ['pytest'],  # Dependencias para pruebas
    },
    entry_points={
        'console_scripts': [
            'movement_node= quadruped_master.movement:main',
            'test_mov= quadruped_master.test_mov:main',
            'joy_to_motion_node=quadruped_master.joy_to_motion:main',
            'communication_gazebo_node= quadruped_master.communication_gazebo:main',
        ],
    },
)

