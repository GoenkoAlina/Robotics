from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'turtle_tf2_carrot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alina',
    maintainer_email='a.goenko@g.nsu.ru',
    description='Learning tf2 with rclpy',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['turtle_tf2_broadcaster = turtle_tf2_carrot.turtle_tf2_broadcaster:main', 
        'turtle_tf2_listener = turtle_tf2_carrot.turtle_tf2_listener:main',
        'carrot_tf2_broadcaster = turtle_tf2_carrot.carrot_tf2_broadcaster:main',
        ],
    },
)
