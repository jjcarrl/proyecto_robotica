from glob import glob 
import os

from setuptools import find_packages, setup

package_name = 'sensors'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch file para sensores
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robotica',
    maintainer_email='j.carreral@uniandes.edu.co',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        'odometry_node = sensors.odometry:main',
        'camera_node = sensors.camera:main',
        'images_node = sensors.images:main',
        'encoders_node = sensors.encoders:main',
        'mpu_node = sensors.mpu:main',
        'vision_bridge_node = sensors.vision_bridge:main',
        'arm_bridge_node = sensors.arm_bridge:main',
        ],
    },
)
