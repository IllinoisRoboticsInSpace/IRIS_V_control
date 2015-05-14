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

4) __rosbuild__: `rosmake <package name>`


## Adding new packages

1) `roscd`

2) `cd IRIS_V_control`

3) Follow the rosbuild package creation tutorial at: <br />
http://wiki.ros.org/ROS/Tutorials/CreatingPackage

## Running the code

### With odroid (over the network)

1) Make sure that your `/etc/hosts` file on the control laptop has an entry in the format <br \>
`<odroid IP address> odroid-desktop`

2) Make sure that the odroid `/etc/hosts` file has an entry in the form <br \>
`<control laptop IP address> <control laptop name>`

3) Make sure XBOX controller is plugged into control laptop

__Note__: First two steps may not be necessary, but I had some issues,
so might as well do it to be safe.

4) `ping odroid-desktop` (make sure you can communicate with the odroid)

5) In a terminal: `ssh odroid@odroid-desktop`

6) `roscd`

7) `cd IRIS_V_control`

8) `roslaunch launch/odroid_control.launch`

9) In another termal tab: `export ROS_MASTER_URI=http://odroid-desktop:11311`

10) Check that you can see topics from odroid: `rostopic echo /IRIS/FSM_trigger` <br\>
You should be getting a "data: False" message every half second. If not, something is
wrong with the network setup. If you're getting it, then kill the echo (`Ctrl+c`)

11) `roscd`

12) `cd IRIS_V_control` (basically make sure you're in the right folder for launch files)

13) `roslaunch launch/control_laptop.launch`

14) In another terminal tab: `ssh odroid@odroid-desktop`

15) `rosrun serial_comm arduino_comm.py` (To start communication with arduino.) <br \>
If you get an error, you may need to unplug and replug the arduino so that it ends up 
on "/dev/TTYACM0".

### Without odroid (do everything on control laptop for testing)

__Note__: The steps telling you to export ROS_MASTER are optional for running the code,
but if you will want to echo any of the topics it becomes necessary.

1) Make sure the XBOX controller is plugged into the control laptop

2) Make sure the arduino is plugged into the control laptop if you want to talk to motors

2) `export ROS_MASTER_URI=http://localhost:11311`

3) `roscd`

4) `cd IRIS_V_control`

5) `roslaunch launch/odroid_control.launch`

6) In a new terminal (and make sure it is in the IRIS\_V\_control folder) <br \>
`export ROS_MASTER_URI=http://localhost:11311`

7) `roslaunch launch/control_laptop.launch`

8) In another new terminal: `export ROS_MASTER_URI=http://localhost:11311`

9) `rosrun serial_comm arduino_comm.py` (To start communication with arduino.) <br \>
If you get an error, you may need to unplug and replug the arduino so that it ends up 
on "/dev/TTYACM0". Skip this step if you're not using the arduino.


## Package list

### IRIS\_msgs

Messages containing data necessary for finite state machine to operate
effectively. Includes descriptions of current and desired robot status.

### manual\_robot\_control

Stuff needed to utilize the Xbox controller for manual control.

### finite\_state\_machine

Provides robot state control based on the robot's status and what actions
need to be taken to effectively complete the mission.

### msg\_relays

Passes odometry and goal data from the IRIS control scheme
to the navigation stack.

### serial\_comm

Serial communication with the Arduino to send control commands
to the EM components and get feedback and localization data.

### obstacle\_detection

Code to utilize the Kinect to detect obstacles in the arena and convert
this data to a point cloud that can be utilized by move\_base.

### launch

Not technically a package. Has the launch files to launch all ROS nodes on
the robot and control laptop for control and navigation purposes. Also all
parameter files needed.
