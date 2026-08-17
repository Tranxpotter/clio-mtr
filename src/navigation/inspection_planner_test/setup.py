from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'inspection_planner_test'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), (glob('launch/*.launch.py'))),
        (os.path.join('share', package_name, 'databases'), []),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='abc',
    maintainer_email='michaelyclaw@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "goal_point_recorder = inspection_planner_test.goal_point_recorder:main",
            "goal_pose_recorder = inspection_planner_test.goal_pose_recorder:main",
            "waypoint_publisher_service = inspection_planner_test.waypoint_publisher_service:main",
            "viewpose_publisher_service = inspection_planner_test.viewpose_publisher_service:main", 
            "inspection_poses_loader = inspection_planner_test.inspection_poses_loader:main", 
            "odometry_saver = inspection_planner_test.odometry_saver:main", 
        ],
    },
)
