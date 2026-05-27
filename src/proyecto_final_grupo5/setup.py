from glob import glob
import os
from setuptools import find_packages, setup

package_name = 'proyecto_final_grupo5'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Grupo 5',
    maintainer_email='j.carreral@uniandes.edu.co',
    description='Proyecto Final IELE3338L — Robot autónomo Grupo 5',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [],
    },
)
