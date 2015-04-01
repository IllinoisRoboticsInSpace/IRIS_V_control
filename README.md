# IRIS\_V\_Control

The finite state machine and related packages.

## Installation

Note: we are messing with the standard ROS configuration to make a catkin 
workspace within the IRIS\_V\_control repository for convenience reasons.

This assumes you have already followed the tutorial to create a workspace
with rosbuild selected, replacing all instances of "fuerte" with "hydro": <br />
http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment

1)  `roscd`

2)  Clone the repository

__ssh__: `git clone git@github.com:IllinoisRoboticsInSpace/IRIS_V_control.git`

__https__: `git clone https://github.com/IllinoisRoboticsInSpace/IRIS_V_control.git`

3)  `rosws set IRIS_V_control`

4)  `source setup.bash`

5)  `cd src`

6)  `catkin_init_workspace`

7)  `cd ..`

8)  `source devel/setup.bash`

It is a good idea to add `source <path to IRIS_V_control>/devel/setup.bash`
to your .bashrc file.


## Compiling

__rosbuild__: `rosmake <package name>`

__catkin__:   `catkin_make`

## Adding new packages

1) `cd IRIS_V_control`

2) `roscreate_pkg <package name>`

3) Make sure the .gitignore contains `src/<package name>`

(See `IRIS_msgs/.gitignore`)

## Package list

### IRIS\_msgs

Messages containing data necessary for finite state machine to operate
effectively. Includes descriptions of current and desired robot status.
