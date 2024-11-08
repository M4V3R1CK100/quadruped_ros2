from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'quadruped_master'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=['quadruped_master', 'quadruped_master.*'],exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name), glob('launch/*.launch.py'))        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='erick',
    maintainer_email='erick@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'movement_node=quadruped_master.movement:main'
        ],
    },
)
