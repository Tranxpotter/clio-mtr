from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'localization_utils'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # (os.path.join('share', package_name, 'launch'), (glob('launch/*.launch.py'))),


    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='iwintern',
    maintainer_email='iltlo@connect.hku.hk',
    description='Innowing clio guide robot project localization util nodes',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pose_estimate_remapper = localization_utils.pose_estimate_remapper:main', 
            'static_odom_publisher = localization_utils.static_odom_publisher:main', 
            'tf_height_remover = localization_utils.tf_height_remover:main', 
            'pointcloud_rotator = localization_utils.pointcloud_rotator:main',
        ],
    },
)
