from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'quadruped_communication'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),  
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='erick',
    maintainer_email='eridamen@espol.edu.ec',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'communication_dynamixel=quadruped_communication.u2d2_communication_r2.py:main'
        ],
    },
)
