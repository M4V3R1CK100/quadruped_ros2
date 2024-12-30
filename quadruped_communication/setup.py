from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'quadruped_communication'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=['quadruped_communication', 'quadruped_communication.*'], exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),  
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='erick',
    maintainer_email='eridamen@espol.edu.ec',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest'],  # Dependencias para pruebas
    },
    entry_points={
        'console_scripts': [
            'dynamixel_communication=quadruped_communication.u2d2_communication_r2.py:main',
            'vrep_communication= quadruped_communication.vrep_communication:main', 

        ],
    },
)
