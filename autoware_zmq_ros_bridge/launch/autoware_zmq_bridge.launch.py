from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
config_file = os.path.join(get_package_share_directory('autoware_zmq_ros_bridge'), 'config', 'autoware_zmq_params.yaml')


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='autoware_zmq_ros_bridge',
            executable='autoware_zmq_listener_node',
            name='autoware_zmq_capnp_bridge_node',
            parameters=[config_file]
        )
    ])
