--- BUILDING ---
Make sure the ROS "Joy" package is propely built in your workspace.
The best way to build this thing is to make a build folder in the main package directory (if it's not there already), cd into it, then run:
$ cmake ../
$ make

You also need to have the curses.h library for keyboard_control, but that should be preinstalled for Ubuntu.

--- RUNNING ---

Make sure you've already started roscore in another terminal (don't need to do this if gazebo is running):

XBOX CONTROLLER:
Make sure controller is connected, type:
$ roslaunch manual_robot_control xbox_control.launch

This will launch both the joy node and the kinect_control node, which translates the joy messages into cmd_vel messages.

KEYBOARD CONTROL:
For some odd reason, you need to run this in order for rosrun to be able to execute this node:
$ source manual_robot_control/build/devel/setup.bash

from the directory above the package (obviously)
Then just do this to run it:
$ rosrun manual_robot_control keyboard_control

WSAD to move, make sure the terminal is in focus, hit cntrl-C to quit.
