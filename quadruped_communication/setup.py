from setuptools import find_packages, setup

package_name = 'quadruped_communication'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='john',
    maintainer_email='jumasaet@espol.edu.ec',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dynamixel_communication=quadruped_communication.u2d2_communication_r2.py:main',
            'vrep_communication= quadruped_communication.vrep_communication:main', 

        ],
    },
)
