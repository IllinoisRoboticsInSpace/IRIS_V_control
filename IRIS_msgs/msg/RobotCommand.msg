# Defines the desired robot state

# The target location
geometry_msgs/Pose goal

# Desired turbine position. (TRUE = DOWN)
std_msgs/Bool turbine_position

# Whether the turbine should be on off. (TRUE = ON)
std_msgs/Bool turbine_state

# Whether the bin should be up or down. (TRUE = UP)
std_msgs/Bool bin_position
