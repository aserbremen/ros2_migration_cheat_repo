import os
import yaml

from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    config_file_path = os.path.join(
        get_package_share_directory('my_pkg'), 'config', 'config.yaml')

    with open(config_file_path, 'r') as config_file:
        config_params = yaml.safe_load(config_file)
        shared_params = config_params['/**']['ros__parameters']
        node1_params = config_params['node1']['ros__parameters']
        node2_params = config_params['node2']['ros__parameters']

    node1 = Node(
        package='my_pkg',
        executable='node1',
        name='node1',
        output='screen',
        parameters=[shared_params, node1_params],
        remappings=['cmd_vel:=/cmd_vel_remap'])

    if node2_params['enable_node2']:
        node2 = Node(
            package='my_pkg',
            executable='node2',
            name='node2',
            output='screen',
            parameters=[shared_params, node2_params])

    launch_description_list = [node1]
    if node2_params['enable_node2']:
        launch_description_list.append(node2)

    return LaunchDescription(launch_description_list)
