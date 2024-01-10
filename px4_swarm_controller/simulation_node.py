# !/usr/bin/env python3

# Import the subprocess and time modules
import rclpy
from rclpy.node import Node
import subprocess
import time


class SimulationScript(Node):
    def __init__(self):
        super().__init__('simulation_script')
        self.declare_parameters(namespace='',
                                parameters=[('nb_vehicles', rclpy.Parameter.Type.INTEGER),
                                            ('drone_model', rclpy.Parameter.Type.STRING),
                                            ('world', rclpy.Parameter.Type.STRING),
                                            ('script', rclpy.Parameter.Type.STRING),
                                            ('target', rclpy.Parameter.Type.STRING),
                                            ('label', rclpy.Parameter.Type.STRING),
                                            ('initial_pose', rclpy.Parameter.Type.STRING)])
        (n, m, w, s, t, l, p) = self.get_parameters(
            ['nb_vehicles', 'drone_model', 'world', 'script', 'target', 'label', 'initial_pose'])
        query_arguments = ' -n ' + str(n) + ' -m ' + m + ' -w ' + w + ' -s ' + s + ' -t ' + t + ' -l '+ l + ' -p ' + p
        # List of commands to run
        commands = [
            # Run the Micro XRCE-DDS Agent
            "MicroXRCEAgent udp4 -p 8888",

            # Run the PX4 SITL simulation
            "cd ~/PX4-Autopilot && /bin/bash ./Tools/simulation/gazebo-classic/sitl_multiple_run.sh" + query_arguments

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
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
