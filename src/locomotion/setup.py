from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'locomotion'
setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),  # ← cambiado
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robotica',
    maintainer_email='j.carreral@uniandes.edu.co',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'rc_control=locomotion.rc_control:main',
            'motor_com=locomotion.motor_command:main',
            'graphical_odom=locomotion.trajectory:main',
            'launch=locomotion.launch:main',
            'motor_control=locomotion.motor_control:main',
        ],
    },
)
