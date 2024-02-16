#!/usr/bin/env python3

__author__ = "Arthur Astier"

import json
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


# TODO: Check for adding parameters on the type of command (position, position+speed) to Adapt it in the
#  OffboardControlMode message in waypoint
def parse_swarm_config(config_file):
    # Extract unique models and their counts
    model_counts = {}
    swarm_config = config_file["swarm"]
    for key, item in swarm_config.items():
        model = item["model"]
        model_counts[model] = model_counts.get(model, 0) + 1
    script = ""
    for key, item in model_counts.items():
        script += key + ":" + str(item) + ","
    script = script[:-1]
    # Extract all initial poses
    initial_poses_string = "\""
    initial_poses_dict = dict()
    for idx, item in enumerate(swarm_config.values()):
        initial_pose = item["initial_pose"]
        initial_poses_dict["px4_" + str(idx + 1)] = initial_pose
        initial_poses_string += str(initial_pose["x"]) + "," + str(initial_pose["y"]) + "|"
    initial_poses_string = initial_poses_string[:-1] + "\""
    # Extract all is_leader values
    is_leaders = [item["is_leader"] for item in swarm_config.values()]
    return len(swarm_config), script, initial_poses_string, initial_poses_dict, is_leaders, config_file["trajectory"]


def generate_launch_description():
    ld = LaunchDescription()
    package_dir = get_package_share_directory('px4_swarm_controller')

    # Extract information from configuration files

    # Swarm information
    with open(os.path.join(package_dir, 'config', 'swarm_config.json'), 'r') as swarm_file:
        swarm_config_data = json.load(swarm_file)
    nb_drones, script, initial_poses, initial_poses_dict, is_leaders, trajectory = parse_swarm_config(swarm_config_data)
    swarm_file.close()

    with open(os.path.join(package_dir, 'config', 'control_config.json'), 'r') as control_file:
        control_config = json.load(control_file)

    # Neighborhood and formation parameters
    neighborhood = control_config["neighborhood"]
    neighbors_exe, neighbors_distance, neighbors_params = neighborhood["neighbors_exe"], \
        neighborhood["neighbor_distance"], neighborhood["params"]

    # Control parameters
    controller_info = control_config["controller"]
    controller_exe, controller_params, is_leader_follower_control = controller_info["controller_exe"], controller_info[
        "params"], controller_info["leader_follower"]
    swarm_file.close()

    if is_leader_follower_control:
        neighbors_params = {"leaders": is_leaders, **neighbors_params}
    else:
        is_leaders = [False for is_leader in is_leaders]
    # Launching simulation
    ld.add_action(
        Node(
            package='px4_swarm_controller',
            executable='simulation_node.py',
            name='simulation_node',
            parameters=[{'script': script, 'initial_pose': initial_poses}]
        )
    )
    xs_init = []
    ys_init = []
    # Add trajectory generator if the drone is a leader and a controller otherwise
    for (namespace, initial_pose), aleader in zip(initial_poses_dict.items(), is_leaders):
        # For all the following nodes, we will switch the x-axis and the y-axis to follow the North East Down
        # convention. In fact, to spawn the drones, we need to follow Gazebo's frame convention East North Up.
        # Therefore, we need to switch the x-axis and the y-axis if we want to respect PX4's conventions.

        xs_init.append(initial_pose["y"])
        ys_init.append(initial_pose["x"])
        if aleader:
            # TODO: pass the type of trajectory as an argument
            ld.add_action(Node(
                package='px4_swarm_controller',
                executable='waypoint',
                name='waypoint',
                namespace=namespace,
                # NED convention has already be taken into account in the node
                parameters=[
                    {"wp_path": os.path.join(package_dir, "config", "Trajectories", trajectory),
                     "x_init": initial_pose["x"], "y_init": initial_pose["y"]}]
            ))
        else:
            ld.add_action(Node(
                package='px4_swarm_controller',
                executable=controller_exe,
                name=controller_exe,
                namespace=namespace,
                # Gains: [Kp_x, Ki_x, Kd_x, Kp_y, Ki_y, Kd_y, Kp_z, Ki_z, Kd_z]
                parameters=[controller_params]
            ))
    ld.add_action(
        Node(
            package='px4_swarm_controller',
            executable=neighbors_exe,
            name='nearest_neighbors',
            namespace='simulation',
            # X,Y,Z formations represents the position wanted in relation to the leader
            parameters=[
                {"nb_drones": nb_drones, "neighbor_distance": neighbors_distance,
                 "x_init": xs_init, "y_init": ys_init, **neighbors_params}]
        ))

    # Arming drones (each drone need to receive offboard command before arming)
    ld.add_action(
        Node(
            package='px4_swarm_controller',
            executable='arming',
            name='arming',
            namespace='simulation',
            parameters=[{"nb_drones": nb_drones}]
        ))

    return ld
