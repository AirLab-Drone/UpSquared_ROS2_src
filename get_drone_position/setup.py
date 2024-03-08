from setuptools import find_packages, setup
import glob
import os

package_name = 'get_drone_position'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='drone',
    maintainer_email='danny1.2104@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test_tf2_broadcast = get_drone_position.test_tf2_broadcast:main',
            'test_calculate_distance = get_drone_position.test_calculate_distance:main',
            'test_reverb = get_drone_position.test_tf2_reverb_Node:main',
        ],
    },
)
