from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'quadruped_app'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name), glob('launch/*.launch.py'))   
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
            'interface_node=quadruped_app.interface:main',
            'interface_example=quadruped_app.interface_example:main',
            'image_node=quadruped_app.streaming:main'
        ],
    },
)
