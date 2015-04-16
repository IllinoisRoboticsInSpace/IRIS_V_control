# This message contains the status of the robot

# The estimated pose (position and orientation) with estimated uncertainty.
geometry_msgs/PoseWithCovariance pose

# The current velocity (linear and angular) of the robot.
geometry_msgs/TwistWithCovariance twist

# Status of the paddle actuator.
# (0 = RETRACTING, 1 = DOWN, 2 = EXTENDING, 3 = UP)
uint32 paddle_position

# Whether or not the paddle is on. (TRUE = ON)
bool paddle_state

# Status of the bin actuator.
# (0 = RETRACTING, 1 = DOWN, 2 = EXTENDING, 3 = UP)
uint32 bin_position

# Load cell status (weight in kg)
float32 load

# If there is a problem detected (e.g. with motor).
string error
