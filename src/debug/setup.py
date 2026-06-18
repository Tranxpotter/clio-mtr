from setuptools import find_packages, setup

package_name = 'debug'

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
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'twist_plotter_node = debug.twist_plotter_node:main',
            'realtime_twist_plotter_node = debug.realtime_twist_plotter_node:main',
            'realtime_twist_stamped_plotter_node = debug.realtime_twist_stamped_plotter_node:main', 
            'path_stability_visualizer_node = debug.path_stability_visualizer_node:main',
        ],
    },
)
