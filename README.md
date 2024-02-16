# PX4 Swarm Controller

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

The aim of this repository is to provide a scalable multi-drone simulation for easy testing of control laws on fleets of
UAVs. The default controller is based on a leader-follower approach taken from the
paper [Distributed leader-follower formation control for
multiple quadrotors with weighted topology (Zhicheng Hou, Isabelle Fantoni)](https://hal.science/hal-01180491/document).

This project was carried out as part of the Research and Development professional option at Ecole Centrale de Nantes
with my partner Martin Piernas.

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
- Move the package containing the messages in the ```/src``` directory:
  ```shell
  mv px4_swarm_controller/custom_msgs/ ~/ros2_ws/src/
  ```
- Build the package and the custom messages using ```colcon```
  ```shell
  colcon build --packages-select custom_msgs
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

#### Neighbors message

You can create your custom ```Neighbors``` message for your custom command law. It should verify the traits specified in
the header ```NeighborsTraits```.

#### Neighborhood

You can handle the neighborhood of your drones by deriving from the class ```NearestNeighbors```. This class is
templated on the type of ```Neighbors``` message defined previously. You can then
override 3 methods:

- ```process_neighbor_position```

Executed when a drone has been detected as a neighbor from another drone.

- ```process_neighborhood```

Executed when the neighborhood of one drone has been computed

- ```enrich_neighborhood```

Executed when all neighborhood of all the drones have been computed.

If you want to see an example, check the ```WeightedTopologyNeighbors``` class.

#### Controller

You can build your controller based on the ```SwarmController``` interface which is templated on the
same ```Neighbors``` message than ```NearestNeighbors```.

You can override the ```neighbors_callback``` and the ```timer_callback``` which is quite explicit.

If you want to see an example, check the ```WeightedTopologyNeighbors``` class.

## Weighted Topology Swarm Controller

### Implementation

- We followed all the algorithms provided in the paper. However, the $\sigma_{n}$ operator wasn't specified but is
  defined
  as follows:

  $$\forall (a,b) \in \mathbb{R}^{2},\sigma_b(a) = sign(a)\times min(|a|,b)$$
- We made a **small improvement** by adding a PID controller on the acceleration of each axis (XYZ).

  Indeed, in the paper, they were using proportional gains to weight the homogeneous term at a position and the
  homogeneous term at a velocity. The output of this weighted sum is an acceleration. Therefore, we directly applied a
  PID
  control on the acceleration command, and we **removed** the proportional gains.

  It allows us to apply this control on the Z-axis whereas in the paper, the drones where assumed in the same plan.

### Results

We tuned the PID controllers only for the swarm and the trajectories provided in this repository.

#### Up and down trajectory

https://github.com/artastier/PX4_Swarm_Controller/assets/76452245/f1ebcacc-f2e5-4039-978f-fa705736e6bd

#### Circular trajectory

https://github.com/artastier/PX4_Swarm_Controller/assets/76452245/57d36f32-e843-49ee-a215-283e57f79523

## Launchfile Configuration

You can change the characteristics of each node only by modifying the following configuration files.

### Swarm configuration

- **model**:
  ```"iris" "plane" "standard_vtol" "rover" "r1_rover" "typhoon_h480"```
- **initial_pose**: Initial coordinates in the North-East plane.
- **is_leader**: Defines if the drone is a leader or a follower for the swarm controller.
- **trajectory**: Provide the name of a YAML file defining waypoints in the ```config/Trajectories``` directory.

```json
{
  "swarm": {
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
        "x": 3.0,
        "y": 0.0
      },
      "is_leader": false
    }
  },
  "trajectory": "waypoints_up_and_down.yaml"
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
```

### Neighborhood and controller

- **neighbors_exe**: Name of the node executable that publish the neighborhood of each drone.
- **neighbor_distance**: Distance under which a drone is considered as a neighbor.
- Neighborhood **params**: Depends on the parameters declared in your custom node.
- Weighted neighbors **x_formation**, **y_formation**, **z_formation**: Distance wanted between each follower and the
  leader in the North
  East Down frame.
- **controller_exe**: Controller node executable.
- **leader_follower**:
    - **true**: Leader-follower controller.
    - **false**: Non leader-follower controller (no Waypoint node will run).
- Controller **params**: Depends on the parameters declared in your custom controller.
- Weighted controller **gains**: Gains for the 3 acceleration PID controllers of ```WeightedTopologyController```in this
  order:

  $$[K_{px},K_{ix},K_{dx},K_{py},K_{iy},K_{dy},K_{pz},K_{iz},K_{dz}]$$

```json

{
  "neighborhood": {
    "neighbors_exe": "weighted_topology_neighbors",
    "neighbor_distance": 3.0,
    "params": {
      "x_formation": [
        1.0,
        0.0,
        0.0
      ],
      "y_formation": [
        0.0,
        0.0,
        2.0
      ],
      "z_formation": [
        0.0,
        0.0,
        -1.0
      ]
    }
  },
  "controller": {
    "controller_exe": "weighted_topology_controller",
    "leader_follower": true,
    "params": {
      "gains": [
        1.0,
        0.0,
        0.0,
        2.5,
        0.0,
        0.0,
        2.75,
        0.1,
        0.001
      ]
    }
  }
}


```

### Launching the simulation

You can run the simulation by launching ```lauch_simulation.py```:

```shell
ros2 launch px4_swarm_controller launch_simulation.py
```

## Improvements

- Our system only works with the **iris** model, as the namespaces used in the launchfile are in the form "/px4_i",
  which is specific to the iris model. We'll need to be able to change the namespace according to the model used in the
  launchfile in order to support all PX4 models.
- Modify the bash script ```sitl_multiple_run.sh``` to use the latest version of gazebo.
- Find a generic setting for the ```WeightedTopologyController``` that works for any swarm.
- Be able to change of swarm controller, trajectory and formation online (without rebuilding the package).
