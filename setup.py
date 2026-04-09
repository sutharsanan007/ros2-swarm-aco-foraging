import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'multi_robot_foraging'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # ADD THESE LINES TO INCLUDE YOUR FOLDERS:
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '*.world'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Multi-robot foraging using stigmergy and ArUco markers',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'unified_agent = multi_robot_foraging.unified_agent:main',
            'aruco_detector = multi_robot_foraging.aruco_detector:main',
            'pheromone_manager = multi_robot_foraging.pheromone_manager:main',
            'pheromone_mapper = multi_robot_foraging.pheromone_mapper:main',
            'foraging_controller = multi_robot_foraging.foraging_controller:main',
        ],
    },
)