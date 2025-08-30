from setuptools import setup
from glob import glob
import os

import setuptools
import numpy

package_name = 'star_solution'

# Prepare data_files list
data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ('share/' + package_name + '/config', glob('config/*')),
    # ('share/' + package_name + '/data', glob('data/*')),
]

# Recursively add ALL files from data/ (only files!)
data_dir = 'data'
if os.path.isdir(data_dir):
    for root, _, files in os.walk(data_dir):
        if not files:
            continue
        rel_dir = os.path.relpath(root, '.')  # e.g., data/tb_office_v02
        install_dir = os.path.join('share', package_name, rel_dir)
        src_files = [os.path.join(root, f) for f in files]
        data_files.append((install_dir, src_files))

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Pub/Sub example with RViz and delayed rosbag play.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'solution1 = star_solution.solution1:main',
            'lidar_transformer = star_solution.lidar_transformer:main'
            # tempo unused 
            # 'cloud_preproc_node = star_solution.cloud_preproc_node:main',
            # 'scan_builder_node = star_solution.scan_builder_node:main',
            # 'odom_node = star_solution.odom_node:main',
            # 'loop_closure_node = star_solution.loop_closure_node:main',
            # 'pose_graph_node = star_solution.pose_graph_node:main',
            # 'map_node = star_solution.map_node:main',
        ],
    },
)
