# This message contains the state of the robot

# The estimated pose (position and orientation) with estimated uncertainty.
geometry_msgs/PoseWithCovariance pose

# The current velocity (linear and angular) of the robot.
geometry_msgs/TwistWithCovariance twist

# Whether or not the turbine is down. (TRUE = DOWN)
std_msgs/Bool turbine_position

# Whether or not the turbine is on. (TRUE = ON)
std_msgs/Bool turbine_state

# Whether the bin is up or down. (TRUE = UP)
std_msgs/Bool bin_position

# Load cell status (weight in kg)
std_msgs/Float32 load

# If there is a problem detected (e.g. with motor).
std_msgs/String error
