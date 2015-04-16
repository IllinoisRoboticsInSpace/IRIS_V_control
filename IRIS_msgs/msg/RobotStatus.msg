# This message contains the status of the robot

# The estimated pose (position and orientation) with estimated uncertainty.
geometry_msgs/PoseWithCovariance pose

# The current velocity (linear and angular) of the robot.
geometry_msgs/TwistWithCovariance twist

# Status of the paddle actuator.
# (0 = RETRACTING, 1 = DOWN, 2 = EXTENDING, 3 = UP)
std_msgs/UInt32 paddle_position

# Whether or not the paddle is on. (TRUE = ON)
std_msgs/Bool paddle_state

# Status of the bin actuator.
# (0 = RETRACTING, 1 = DOWN, 2 = EXTENDING, 3 = UP)
std_msgs/UInt32 bin_position

# Load cell status (weight in kg)
std_msgs/Float32 load

# If there is a problem detected (e.g. with motor).
std_msgs/String error
