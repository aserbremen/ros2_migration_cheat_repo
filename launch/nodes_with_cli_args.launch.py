import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import OpaqueFunction, DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwards):
    config_file_path = os.path.join(
        get_package_share_directory('my_pkg'), 'config', 'config.yaml')

    with open(config_file_path, 'r') as config_file:
        config_params = yaml.safe_load(config_file)
        shared_params = config_params['/**']['ros__parameters']
        node1_params = config_params['node1']['ros__parameters']
        node2_params = config_params['node2']['ros__parameters']
    shared_params = overwrite_yaml_params_from_cli(shared_params, context.launch_config)
    node1_params = overwrite_yaml_params_from_cli(node1_params, context.launch_config)
    node2_params = overwrite_yaml_params_from_cli(node2_params, context.launch_config)

    # Declare the nodes with the updated parameters ...
    node1 = Node(
        package='my_pkg',
        executable='node1',
        name='node1',
        output='screen',
        parameters=[shared_params, node1_params])

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


# Param type mapping used to overwrite yaml params
param_mapping = {
    'use_sim_time': bool,
    'topic2': str,
}


def overwrite_yaml_params_from_cli(yaml_params, cli_params):
    for key, value in cli_params.items():
        if key in yaml_params and value != '':
            # infer the correct data type, since all cli params are strings
            yaml_params[key] = param_mapping[key](value)
            # Overwrite the boolean values, non empty strings are always True
            if value == 'true' or value == 'True':
                yaml_params[key] = True
            elif value == 'false' or value == 'False':
                yaml_params[key] = False
    return yaml_params


def generate_launch_description():
    launch_description_list = []
    # Declare the cli arguments
    for key, value in param_mapping.items():
        launch_description_list.append(
            DeclareLaunchArgument(key, default_value=''))
    launch_description_list.append(OpaqueFunction(function=launch_setup))
    return LaunchDescription(launch_description_list)
