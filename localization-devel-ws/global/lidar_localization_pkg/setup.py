import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'lidar_localization_pkg'

setup(
    name=package_name,
    version='0.0.0',
    # Packages to export
    packages=find_packages(exclude=['test']),
    # Files we want to install, specifically launch files
    data_files=[
        # Install marker file in the package index
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # Include our package.xml file
        (os.path.join('share', package_name), ['package.xml']),
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=[
        'setuptools',
        'rclpy',
        'geometry_msgs',
        'obstacle_detector',
        'visualization_msgs',
        'std_msgs',
        'tf2_ros',
    ],
    zip_safe=True,
    maintainer='jossiew621',
    maintainer_email='jossiew621@gapp.nthu.edu.tw',
    keywords=['scan', 'beacon', 'localization'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache-2.0 License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Eurobot localization using LiDAR scan, probability and triangulation',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_localization = lidar_localization_pkg.lidar_member_function:main',
            'circle_publisher = lidar_localization_pkg.probability_circle_publisher:main',
            'pred_publisher = lidar_localization_pkg.pred_publisher:main',
        ],
    },
)
