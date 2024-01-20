__author__ = "Arthur Astier"

import json
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def parse_swarm_config(config_file):
    # Extract unique models and their counts
    model_counts = {}
    for key, item in config_file.items():
        model = item["model"]
        model_counts[model] = model_counts.get(model, 0) + 1
    script = ""
    for key, item in model_counts.items():
        script += key + ":" + str(item) + ","
    script = script[:-1]
    # Extract all initial poses
    initial_poses = "\""
    for item in config_file.values():
        initial_pose = item["initial_pose"]
        initial_poses += str(initial_pose["x"]) + "," + str(initial_pose["y"]) + "|"
    initial_poses = initial_poses[:-1] + "\""
    # Extract all is_leader values
    is_leaders = [item["is_leader"] for item in config_file.values()]
    return len(config_file), script, initial_poses, is_leaders


def generate_launch_description():
    package_dir = get_package_share_directory('px4_swarm_controller')

    # Extract information from configuration files

    # Swarm information
    with open(os.path.join(package_dir, 'config', 'swarm_config.json'), 'r') as file:
        swarm_config_data = json.load(file)
    nb_drones, script, initial_poses, is_leaders = parse_swarm_config(swarm_config_data)

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
            executable='waypoint',
            name='waypoint',
            namespace='px4_1',
            parameters=[
                {"wp_path": os.path.join(package_dir, "config", "waypoints.yaml"), "x_init": 0.0, "y_init": 1.0}]
        ),
        Node(
            package='px4_swarm_controller',
            executable='leader_control',
            name='leader_control',
            namespace='px4_1'
        )
    ])