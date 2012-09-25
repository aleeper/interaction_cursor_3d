# When the cursor attaches to an interactive_marker, this
# feedback message contains the frame_id of the marker
# and the pose of the cursor relative to that frame.
geometry_msgs/PoseStamped pose

# 
uint8 event_type

# KEEP_ALIVE: should probably be sent regularly by Rviz, otherwise
# the interaction app should assume that it has lost control of the marker.
uint8 KEEP_ALIVE = 0
# others?



