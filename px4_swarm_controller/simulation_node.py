#!/usr/bin/env python3

# Import the subprocess and time modules
import rclpy
from rclpy.node import Node
import subprocess
import time


class SimulationScript(Node):
    def __init__(self):
        super().__init__('simulation_node')
        self.declare_parameter('nb_vehicles', 0)
        self.declare_parameter('drone_model', '')
        self.declare_parameter('world', '')
        self.declare_parameter('script', '')
        self.declare_parameter('target', '')
        self.declare_parameter('label', '')
        self.declare_parameter('initial_pose', '')
        (n, m, w, s, t, l, p) = self.get_parameters(
            ['nb_vehicles', 'drone_model', 'world', 'script', 'target', 'label', 'initial_pose'])
        query_arguments = {' -n ': n.value, ' -m ': m.value, ' -w ': w.value, ' -s ': s.value, ' -t ': t.value, ' -l ': l.value,
                           ' -p ': p.value}
        query = ""
        for key, value in query_arguments.items():
            if value:
                if type(value) is str:
                    query += key + value
                else:
                    query += key + str(value)
        self.get_logger().info("Provided parameters" + query)
        # List of commands to run
        commands = [
            # Run the Micro XRCE-DDS Agent
            "MicroXRCEAgent udp4 -p 8888",

            # Run the PX4 SITL simulation
            "cd ~/PX4-Autopilot && /bin/bash ./Tools/simulation/gazebo-classic/sitl_multiple_run.sh" + query

        ]

        # Loop through each command in the list
        for command in commands:
            # Each command is run in a new tab of the gnome-terminal
            subprocess.run(["gnome-terminal", "--tab", "--", "bash", "-c", command + "; exec bash"])

            # Pause between each command
            time.sleep(1)


def main(args=None):
    rclpy.init(args=args)
    node = SimulationScript()
    try:
        rclpy.spin(node)
    finally:
        subprocess.run("pkill gnome-terminal", shell=True)
        rclpy.shutdown()


if __name__ == "__main__":
    main()
