from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'panda_can_rcv'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'panda_msgs'), glob('msg/*.msg')),
        (os.path.join('share', package_name, 'autoware_vehicle_msgs'), glob('msg/*.msg')),  
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='avinashumeshsarma',
    maintainer_email='avinashumeshsarma@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'can_listener_node = panda_can_rcv.can_listener_node:main',
        ],
    },
)
