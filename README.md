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

## Compiling

1) `roscd`

2) `source setup.bash` (needed if you just pulled from the repo or added packages)

3) `cd IRIS_V_control`

4a) __rosbuild__: `rosmake <package name>`


## Adding new packages

### rosbuild

1) `roscd`

2) `cd IRIS_V_control`

3) Follow the rosbuild package creation tutorial at: <br />
http://wiki.ros.org/ROS/Tutorials/CreatingPackage

4) Make sure there is a .gitignore in the package path with `src/<package name>`

(See `IRIS_msgs/.gitignore`)

## Package list

### IRIS\_msgs

Messages containing data necessary for finite state machine to operate
effectively. Includes descriptions of current and desired robot status.

### manual\_robot\_control

Stuff needed to make utilize the Xbox controller for manual control.

### finite\_state\_machine

Provides robot state control.

__Subcribed__:

/joy - Xbox controller command
/IRIS/status - Robot status from serial comm node
/cmd\_vel - Robot command velocity from move\_base


__Plublished__:

/IRIS/command - Robot command to serial comm node
