# Defines the desired robot state

# The target location
geometry_msgs/Pose goal

# Desired paddle position. (TRUE = DOWN)
std_msgs/Bool paddle_position

# Whether the paddle should be on off. (TRUE = ON)
std_msgs/Bool paddle_status

# Whether the bin should be up or down. (TRUE = UP)
std_msgs/Bool bin_position
