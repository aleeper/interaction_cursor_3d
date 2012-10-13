# When the cursor attaches to an interactive_marker, this feedback 
# message contains the frame_id of the TF frame that the marker 
# is used to "control", if any. The pose records the pose of the 
# cursor relative to that frame at time of grasp.
geometry_msgs/PoseStamped pose

# Default for both event_type and attachment_type
uint8 NONE = 0

# Attachment types define what kind of virtual spring coupling there 
# is between the device and the grabbed object frame.
uint8 POSITION = 1
uint8 ORIENTATION = 2
uint8 POSITION_AND_ORIENTATION = 3
uint8 attachment_type

# KEEP_ALIVE: should probably be sent regularly by Rviz, otherwise
# the interaction app should assume that it has lost control of the marker.
uint8 GRABBED = 1
uint8 KEEP_ALIVE = 2
uint8 RELEASED = 3
uint8 LOST_GRASP = 4

uint8 event_type




