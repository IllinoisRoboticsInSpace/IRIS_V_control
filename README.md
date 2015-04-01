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

1) `roscd`

2) `cd IRIS_V_control`

3a) __rosbuild__: `rosmake <package name>`

3b) __catkin__:   `catkin_make`

## Adding new packages

### rosbuild

1) `roscd`

2) `cd IRIS_V_control`

3) Follow the rosbuild package creation tutorial at: <br />
http://wiki.ros.org/ROS/Tutorials/CreatingPackage

4) Make sure there is a .gitignore in the package path with `src/<package name>`

(See `IRIS_msgs/.gitignore`)

### catkin

1) `roscd`

2) `cd IRIS_V_control/src`

3) Follow the catkin package creation tutorial at <br />
http://wiki.ros.org/ROS/Tutorials/CreatingPackage

## Package list

### IRIS\_msgs

Messages containing data necessary for finite state machine to operate
effectively. Includes descriptions of current and desired robot status.
