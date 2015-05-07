# This message contains the status of the robot

# Odometry estimate (position, orientation, velocities, frames)
nav_msgs/Odometry odom

# Status of the paddle actuator.
# (0 = RETRACTING, 1 = DOWN, 2 = EXTENDING, 3 = UP)
uint32 paddle_position

# Whether or not the paddle is on. (TRUE = ON)
bool paddle_status

# Status of the bin actuator.
# (0 = RETRACTING, 1 = DOWN, 2 = EXTENDING, 3 = UP)
uint32 bin_position

# Load cell status (weight in kg)
float32 load

# If there is a problem detected (e.g. with motor).
string error
