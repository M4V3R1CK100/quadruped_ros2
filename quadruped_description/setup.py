from setuptools import setup
import os
from glob import glob

package_name = 'quadruped_description'

models_paths = []

directories= glob('models/')+glob('models/*/')+glob('models/*/*/')
for directory in directories:
    models_paths.append((os.path.join('share',package_name,directory),glob(f'{directory}/*.*')))

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']), 
        #Add all xacro and URDF files that descripe the robot to the share directory
        (os.path.join('share',package_name,'urdf'),glob('urdf/*.xacro')),
        #Adding visual and collision mehses to the share directory
        (os.path.join('share',package_name,'meshes'),glob('meshes/*.stl')),
        #Adding Rviz2 configuration files
        (os.path.join('share',package_name,'rviz'),glob('rviz/*.rviz')),
        #Adding launch files
        (os.path.join('share',package_name,'launch'),glob('launch/*.launch.*')),
        (os.path.join('share',package_name,'config'),glob('config/*.yaml')),

        # (os.path.join('share',package_name,'worlds'),glob('worlds/*')),
        ]+models_paths,

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='erick',
    maintainer_email='eridamen@espol.edu.ec',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)

