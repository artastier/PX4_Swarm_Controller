# PX4 Swarm Controller

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

The aim of this repository is to provide a scalable multi-drone simulation for easy testing of control laws on fleets of
UAVs. The default controller is based on a leader-follower approach taken from the paper [Distributed leader-follower formation control for
multiple quadrotors with weighted topology (Zhicheng Hou, Isabelle Fantoni)](https://hal.science/hal-01180491/document).

## Install

### ROS2

This package was written using ROS2 Humble under Ubuntu 22.04. If it's not installed, check
the [ROS2 documentation](https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html) or run
the commands
below.

```shell
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update && sudo apt upgrade -y
sudo apt install ros-humble-desktop
sudo apt install ros-dev-tools
```

Don't forget to source your installation in the terminal or in your ```bashrc``` (See example below).

```shell
source /opt/ros/humble/setup.bash
```

You will also need to have ```gazebo``` [installed](https://classic.gazebosim.org/tutorials?tut=install_ubuntu).

### PX4 Autopilot

We used the PX4 Autopilot which uses the Micro XRCE-DDS Agent & Client as middleware.

Here are the installation commands to run in your terminal, you can also check
the [PX4 documentation](https://docs.px4.io/main/en/ros/ros2_comm.html):

```shell
cd
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
cd PX4-Autopilot/
make px4_sitl

pip install --user -U empy==3.3.4 pyros-genmsg setuptools

git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build
cd build
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib/
```

### Install and build the package

- Create a ROS2 workspace (Skip if already done)
    ```shell
    cd
    mkdir -p ros2_ws/src
    ```
- Then clone this repository (ROS2 package) in ```ros2_ws/src``` and rename the directory as ```px4_swarm_controller```
  ```shell
  cd ros2_ws/src
  git clone https://github.com/artastier/PX4_Swarm_Controller.git
  mv PX4_Swarm_Controller px4_swarm_controller
  ```
- We need to ```overwrite``` the original ```sitl_multiple_run.sh```  of PX4 by the new one
  ```shell
  mv -i px4_swarm_controller/sitl_multiple_run.sh ~/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_multiple_run.sh
  ```
- Build the package using ```colcon```
  ```shell
  colcon build --packages-select px4_swarm_controller
  ```
- Don't forget to source your ROS2 workspace in your terminal or in your ```bashrc``` (See example below).
  ```shell
  source ~/ros2_ws/install/local_setup.bash
  ```

Then you can run the simulation by launching ```lauch_simulation.py```:

```shell
ros2 launch px4_swarm_controller launch_simulation.py
```

## Configuration

### Swarm configuration

- **model**:
  ```"iris" "plane" "standard_vtol" "rover" "r1_rover" "typhoon_h480"```
- **initial_pose**: Initial coordinates in the North-East plane.
- **is_leader**: Defines if the drone is a leader or a follower for the swarm controller.

```json
{
  "1": {
    "model": "iris",
    "initial_pose": {
      "x": 0.0,
      "y": 1.0
    },
    "is_leader": true
  },
  "2": {
    "model": "iris",
    "initial_pose": {
      "x": 1.0,
      "y": 0.0
    },
    "is_leader": false
  },
  "3": {
    "model": "iris",
    "initial_pose": {
      "x": 0.0,
      "y": -1.0
    },
    "is_leader": false
  },
  "4": {
    "model": "iris",
    "initial_pose": {
      "x": -1.0,
      "y": 0.0
    },
    "is_leader": false
  }
}
```

### Waypoints configuration

Waypoints are stored in the ```config/waypoints.yaml``` file.

They should be express in the **North-East-Down** frame. The ```waypoint``` node loops over the waypoints given for one
drone.

This node is launched if and only if the ```is_leader``` variable is set to ```true``` in the **Swarm
configuration file**. The key in the YAML file should correspond to the namespace of the drone you set
the ```is_leader``` variable to ```true``` (See example below).

You can also provide **2 thresholds**, the distance and the angle. When both the cartesian error and the angle error are
below these thresholds, the ```waypoint``` node sends the next setpoint.

```yaml
threshold: 0.1
threshold_angle: 0.4
# Coordinates are in the North east down frame x -> y and y -> x
wp:
  /px4_1:
    # Up and down
    - { x: 0, y: 1, z: -5, yaw: 0 }
    - { x: 0, y: 1, z: -1, yaw: 0 }
  /px4_2:
    # Up and down
    - { x: 1, y: 0, z: -5, yaw: 0 }
    - { x: 1, y: 0, z: -1, yaw: 0 }
  /px4_3:
    # Up and down
    - { x: 0, y: -1, z: -5, yaw: 0 }
    - { x: 0, y: -1, z: -1, yaw: 0 }
  /px4_4:
    # Up and down
    - { x: -1, y: 0, z: -5, yaw: 0 }
    - { x: -1, y: 0, z: -1, yaw: 0 }
```

### Controller configuration

Coming soon

## Usage

### Arming drones

To be able to switch to offboard mode (i.e Sending setpoints using ROS2 topics) the drone needs to
receive ```OffboardControlMode``` messages both <u>before</u> trying to <u>switch to offboard mode</u> and <u>arming</u>
the drone.

(See [ROS2 Offboard Control Example](https://docs.px4.io/main/en/ros/ros2_offboard_control.html#implementation))

The ```arming``` node aims to switch to offboard mode and arm all the drones in the simulation. Once it's done it will
shut down.

Given the ```number of drones``` as a parameter and **assuming** all the namespaces are standardized
with ```/px4_<instance_id>```, you can launch this node by adding the following code in your launch description:

```python
from launch_ros.actions import Node

Node(
    package='px4_swarm_controller',
    executable='arming',
    name='arming',
    namespace='simulation',
    parameters=[{"nb_drones": nb_drones}]
)
```

### Create your own control law
#### Neighbors
3 stages
- Neighbor
- Neighborhood
- Neighborhood of each drone calculated

#### Controller

### Launching the simulation

You can run the simulation by launching ```lauch_simulation.py```:

```shell
ros2 launch px4_swarm_controller launch_simulation.py
```