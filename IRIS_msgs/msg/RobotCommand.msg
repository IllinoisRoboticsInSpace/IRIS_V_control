# Defines the desired robot state

# The target location
geometry_msgs/Pose goal

# Desired paddle position. (TRUE = DOWN)
bool paddle_position

# Whether the paddle should be on off. (TRUE = ON)
bool paddle_status

# Whether the bin should be up or down. (TRUE = UP)
bool bin_position

# Whether goals should be sent to move_base
bool active_goal

