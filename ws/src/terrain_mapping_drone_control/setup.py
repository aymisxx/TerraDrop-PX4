from setuptools import setup
from glob import glob
import os

package_name = 'terrain_mapping_drone_control'


def package_files(directory):
    paths = []
    for path, _, filenames in os.walk(directory):
        files = [os.path.join(path, f) for f in filenames]
        if files:
            install_path = os.path.join('share', package_name, path)
            paths.append((install_path, files))
    return paths


data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ('share/' + package_name + '/config', glob('config/*')),
]

data_files += package_files('models')
data_files += package_files('scripts')


setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ayushman Mishra',
    maintainer_email='aymisxx@proton.me',
    description=(
        'ROS 2 / PX4 / Gazebo package for autonomous UAV search, '
        'RGB-D cylinder detection, ArUco-guided target selection, '
        'and precision landing in simulation.'
    ),
    license='CC-BY-NC-SA-4.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cylinder_landing_node = terrain_mapping_drone_control.cylinder_landing_node:main',
            'aruco_tracker = terrain_mapping_drone_control.aruco_tracker:main',
            'feature_tracker = terrain_mapping_drone_control.feature_tracker:main',
            'geometry_tracker = terrain_mapping_drone_control.geometry_tracker:main',
            'pose_visualizer = terrain_mapping_drone_control.pose_visualizer:main',
            'spiral_trajectory = terrain_mapping_drone_control.spiral_trajectory:main',
            'auto_detect_land = terrain_mapping_drone_control.auto_detect_land:main',
        ],
    },
    python_requires='>=3.10',
)