from setuptools import find_packages, setup

package_name = 'lidar_calibration'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Progress Munoriarwa, Anders Smitterberg',
    maintainer_email='mmunoria@mtu.edu',
    description='ROS2 node for LiDAR intrinsic parameter estimation using Welford\'s online algorithm to compute running mean, sigma_hit, and outlier rate.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'calibration_node = lidar_calibration.calibration_node:main'
        ],
    },
)
