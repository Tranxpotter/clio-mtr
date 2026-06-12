from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), (glob('launch/*.launch.py'))),
        (os.path.join('share', package_name, 'config'), (glob('config/*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='iwintern',
    maintainer_email='iltlo@connect.hku.hk',
    description='Bringup launch files for clio bot',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
