__author__ = "Arthur Astier"

import json
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_dir = get_package_share_directory('px4_swarm_controller')

    # Extract information from configuration files

    # Swarm information
    with open(os.path.join(package_dir, 'launch', 'swarm_config.json'), 'r') as file:
        swarm_config_data = json.load(file)
    # Extract unique models and their counts
    model_counts = {}
    for key, item in swarm_config_data.items():
        model = item["model"]
        model_counts[model] = model_counts.get(model, 0) + 1
    script = ""
    for key, item in model_counts.items():
        script += key + ":" + str(item) + ","
    script = script[:-1]
    # Extract all initial poses
    initial_poses = "\""
    for item in swarm_config_data.values():
        initial_pose = item["initial_pose"]
        initial_poses += str(initial_pose["x"]) + "," + str(initial_pose["y"]) + "|"
    initial_poses = initial_poses[:-1] + "\""
    # Extract all is_leader values
    is_leaders = [item["is_leader"] for item in swarm_config_data.values()]

    return LaunchDescription([
        # Weird, the simulation_node.py Python script isn't really installed as an executable even if we installed it
        # through the CMakeLists.txt.
        Node(
            package='px4_swarm_controller',
            executable='simulation_node.py',
            name='simulation_node',
            parameters=[{'script': script, 'initial_pose': initial_poses}]
        ),
        Node(
            package='px4_swarm_controller',
            executable='leader_control',
            name='leader_control',
            namespace='px4_1'
        ),
        Node(
            package='px4_swarm_controller',
            executable='leader_control',
            name='leader_control',
            namespace='px4_2'
        ),
        Node(
            package='px4_swarm_controller',
            executable='leader_control',
            name='leader_control',
            namespace='px4_3'
        ),
        Node(
            package='px4_swarm_controller',
            executable='leader_control',
            name='leader_control',
            namespace='px4_4'
        )
    ])
