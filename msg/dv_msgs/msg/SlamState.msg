std_msgs/Header header
string child_frame_id
geometry_msgs/PoseWithCovariance pose
geometry_msgs/TwistWithCovariance twist

#MAP
dv_msgs/IndexedTrack track

# Trackdrive / SLAM state
bool firstlap_done
bool stopping_flag
int32 lap_count
int32 cones_count_actual # perception cones at this instant
int32 cones_count_all # all SLAM cones seen until now
