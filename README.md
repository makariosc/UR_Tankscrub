## Quickstart

```
source ur_driver_ws/devel/setup.sh

roslaunch ur_robot_driver ur5_bringup.launch robot_ip:=192.168.56.25

roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch

roslaunch ur5_moveit_config moveit_rviz.launch rviz_config:=$(rospack find ur5_moveit_config)/launch/moveit.rviz

```

## Controller Setup

Note: This setup was initially built and tested on an **Ubuntu 18.04** system running **ROS Melodic**. Your milage ~~may~~ _will_ vary if using a different setup.

- Install [`moveit`](http://docs.ros.org/en/melodic/api/moveit_tutorials/html/doc/getting_started/getting_started.html) with `sudo apt install ros-melodic-moveit`

- Install [`ur_robot_driver`](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver),
following the build instructions in its README.md. 

After installing `ur_robot_driver`, open `fmauch_universal_robot/<robot>_moveit_config/joint_limits.yaml` and add the following under every joint:

```
    max_position: 3.1415926535897931
    min_position: -3.1415926535897931 
```

So for example, the `elbow_joint` would look like:
```
  elbow_joint:
    has_velocity_limits: true
    max_velocity: 3.1415926535897931
    has_acceleration_limits: false
    max_acceleration: 0
    max_position: 3.1415926535897931
    min_position: -3.1415926535897931 
```

The above joint limits are important because they prevent the trajectory planner from freaking out when planning with the original [-2pi, 2pi] joint limits of a UR robot.

(When editing `joint_limits.yaml`, be sure to keep spacing correct-- and with spaces, not tabs. YAML is finnicky like that.)

## URSim Setup

Through your VM manager (this guide was written using Virtual Box), setup a host-only network with an IP Address/mask of 192.168.56.1/24 (this address was chosen arbitrarily-- the important part is to be consistent).

Download and install the simulation sofware provided by URSim [via this link](https://www.universal-robots.com/download/software-cb-series/simulator-non-linux/offline-simulator-cb-series-non-linux-ursim-3143/) and install it as a virtual machine.

Next, edit the `/etc/network/interfaces` file to assign a static IP to the simulation VM.

Install the [External Control URCap](https://github.com/UniversalRobots/Universal_Robots_ExternalControl_URCap) on your virtual machine. Ensure that the IP set in the External Control setup matches both the static IP as well as the netmask of the host-only network.

## UR Robot Setup
Since URSim and the physical UR Plant are essentially interchangable (both communicate via internet to the controller), all that is needed to swap the Simulator for the Plant is to install the External Control URCap on the physical robot and set the IP's accordingly.

## Starting the ROS/Control Stack

Run each of the following commands in a separate terminal:

```
// Starts `ur_robot_driver` to bride communications between ROS and the UR Plant
roslaunch ur_robot_driver ur5_bringup.launch robot_ip:=<UR_PLANT_IP>

// Starts Moveit interfaces for trajectory planning with a ur5
roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch

// Starts Moveit visualization configs.
roslaunch ur5_moveit_config moveit_rviz.launch rviz_config:=$(rospack find tankscrub)/launch/moveit.rviz
```

At this point, you should start start the `external_control` program on your UR plant.

Finally, to start the controller, run the following in a new terminal:
```
roslaunch tankscrub hardcoded_planner
```

Note that you may need to stop and start the `external_control` URCap between trajectory executions-- no clue why. It's a bit quirky.
Furthermore, this controller is probably a bit dangerous to run on a real robot-- Moveit's cartesian path planning is a bit wonky.

## Gazebo

// Untested

To run the appropriate Gazebo simulation, run
```
roslaunch ur_gazebo <robot_model>_bringup.launch
```
where <robot_model> is either `3`, `5`, `5e`, etc.

When launching Moveit with Gazebo, it should not be necessary to start `ur_robot_driver`, since Moveit can communicate with Gazebo directly. 
