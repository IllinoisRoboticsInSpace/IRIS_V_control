# Defines the desired robot state

# The target location
geometry_msgs/Pose goal

# Desired paddle position. (TRUE = DOWN)
bool paddle_position

# Whether the paddle should be on off. (TRUE = ON)
bool paddle_status

# Whether the bin should be up or down. (TRUE = UP)
bool bin_position

# Whether the robot is mining (limit max speed to half in that case)
bool mining

# The velocity command to be sent to the robot
geometry_msgs/Twist cmd_vel
