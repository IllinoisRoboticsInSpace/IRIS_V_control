# IRIS\_V\_Control

The finite state machine and related packages.

## Installation
1)  Go to your ROS workspace directory (e.g. `cd ~/hyrdro_workspace`)

2)  Clone the repository

__ssh__: `git clone git@github.com:IllinoisRoboticsInSpace/IRIS_V_control.git`

__https__: `git clone https://github.com/IllinoisRoboticsInSpace/IRIS_V_control.git`

3)  `rosws set IRIS_V_control`

4)  `source setup.bash`

## Compiling
`rosmake <package name>`

## Adding new packages
1) `cd IRIS_V_control`

2) `roscreate_pkg <package name>`

3) Make sure the .gitignore contains `src/<package name>`

(See "IRIS\_msgs/.gitignore")t

## Package list

### IRIS\_msgs

Messages containing data necessary for finite state machine to operate
effectively. Includes descriptions of current and desired robot states.

Note: "state" here is not the same as a finite state machine "state"
