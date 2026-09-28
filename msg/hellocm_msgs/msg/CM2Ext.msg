#
# Message from CarMaker ROS Node to external ROS Node
#

# General ROS Header (optional)
std_msgs/Header header

# ROS time when message was sent (optional)
builtin_interfaces/Time time

# Cycle number since simulation start (optional)
int64 cycleno

# Synthetic delay to check synchronization (optional)
float64 synthdelay

float64[3] acc

float64[3] rot

float64[4] wspeed

geometry_msgs/Pose pose
