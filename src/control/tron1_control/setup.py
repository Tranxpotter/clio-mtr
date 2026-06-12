from setuptools import find_packages, setup

package_name = 'tron1_control'

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
    maintainer='iwintern',
    maintainer_email='iltlo@connect.hku.hk',
    description='TRON 1 Control Package',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'bridge = tron1_control.bridge:main',
        ],
    },
)
