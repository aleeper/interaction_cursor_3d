# When the cursor attaches to an interactive_marker, this feedback 
# message contains the frame_id of the TF frame that the marker 
# is used to "control", if any. The pose records the pose of the 
# cursor relative to that frame at time of grasp.
geometry_msgs/PoseStamped pose


# KEEP_ALIVE: should probably be sent regularly by Rviz, otherwise
# the interaction app should assume that it has lost control of the marker.
uint8 NONE = 0
uint8 GRABBED = 1
uint8 KEEP_ALIVE = 2
uint8 LOST_GRASP = 3

uint8 event_type



