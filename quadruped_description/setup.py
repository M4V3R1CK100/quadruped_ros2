from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'quadruped_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name,'urdf'), glob('urdf/*')),
        (os.path.join('share',package_name,'launch'), glob('launch/*.py')),
        (os.path.join('share',package_name,'meshes'), glob('meshes/*')),
        (os.path.join('share',package_name,'rviz'), glob('rviz/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='erick',
    maintainer_email='erick@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest'],  # Dependencias para pruebas
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
