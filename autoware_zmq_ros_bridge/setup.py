from setuptools import find_packages, setup

package_name = 'autoware_zmq_ros_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/autoware_zmq_bridge.launch.py']),
        ('share/' + package_name + '/config', ['config/autoware_zmq_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='avinashumeshsarma',
    maintainer_email='avinashumeshsarma@gmail.com',
    description='ROS 2 - ZMQ bridge using CapnProto',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'autoware_zmq_listener_node = autoware_zmq_ros_bridge.autoware_zmq_listener_node:main'
        ],
    },
)
