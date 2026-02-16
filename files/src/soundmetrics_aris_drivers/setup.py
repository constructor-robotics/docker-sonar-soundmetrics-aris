from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'soundmetrics_aris_drivers'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robotics',
    maintainer_email='eecs-robotics@lists.jacobs-university.de',
    description='ROS2 driver for Sound Metrics ARIS 3000 forward-looking sonar',
    license='MIT',
    entry_points={
        'console_scripts': [
            'soundmetrics_aris3000 = soundmetrics_aris_drivers.soundmetrics_aris3000:main',
            'polar_to_cartesian = soundmetrics_aris_drivers.polar_to_cartesian:main',
        ],
    },
)
