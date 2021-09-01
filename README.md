# MIRRAX Simulator

This is a gazebo simulator for the MIRRAX robot. The robot has a number of simulated sensors which can be configured in the `mirrax_description` package. 

This package provides a simple PS4 joystick controller for manual control of the robot.

## Setup
The setup assumes that you are working in the directory `catkin_ws`. Modify this reference if it is different. 

First, clone the repository to your `catkin_ws` workspace:

```
git clone https://github.com/EEEManchester/mirrax_sim.git ~/catkin_ws/src
```

To install ros dependencies:

```bash
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
```

This package should be built with [catkin_tools](https://catkin-tools.readthedocs.io/en/latest/installing.html)

```bash
cd ~/catkin_ws
catkin build
source devel/setup.bash
```

## Quick Start Guide

There are two robot descriptions available. The `mirrax_description` is the actual MIRRAX robot, while the `urax_desciption` is a half-sized version of MIRRAX built using off-the-shelf components instead of custom parts for the former. The default launch file is set to the actual robot i.e. `mirrax`. The Gazebo simulation for either of them can be invoked as follows:

```bash
roslaunch mirrax_gazebo empty_world.launch
roslaunch mirrax_gazebo empty_world.launch robot:=urax
```

To control the robot using joystick, run the following in a new terminal:

```bash
roslaunch mirrax_simple_demo joystick.launch
```

The controller for the robot are as follows:
| Input                 | Function                                  |
|-----------------------|-------------------------------------------|
| Left joystick         | Robot linear motion                       |
| Right joystick        | Robot yaw                                 |
| Left/Right arrow      | Arm yaw                                   |
| Up/Down arrow         | Arm pan                                   |
| L1                    | Back leg (J5) clockwise rotation          |
| L2                    | Back leg (J5) counter-clockwise rotation  |
| R1                    | Front leg (J6) clockwise rotation         |
| R2                    | Front leg (J6) counter-clockwise rotation |
| Share                 | Linear actuator down                      |
| Options               | Linear actuator up                        |
| Square                | Robot U-shape (default configuration)     |

To view the robot's motion or sensor in RViZ:

```bash
roslaunch mirrax_gazebo rviz.launch
```

