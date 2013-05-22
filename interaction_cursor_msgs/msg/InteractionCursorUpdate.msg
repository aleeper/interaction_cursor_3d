# REQUIRED: Pose for cursor
geometry_msgs/PoseStamped pose

# OPTIONAL: One or more markers to draw for the cursor
#           (if empty, just draws a sphere)
# On creation, marker poses can be specified in any frame, but they will be
# transformed and thereafter "rigidly attached" to the cursor frame.
visualization_msgs/Marker[] markers

# REQUIRED  
uint8 button_state
uint8 NONE        = 0 # No buttons are pressed.
uint8 KEEP_ALIVE  = 1 # Sent while dragging to keep up control of the marker.
uint8 GRAB        = 2 # Start dragging a marker (e.g. a "mouse_down" event).
uint8 RELEASE     = 3 # Stop dragging a marker (e.g. a "mouse_up" event)
uint8 QUERY_MENU  = 4 # Request menu options (e.g. a "right-click).

# REQUIRED  
uint8 key_event
# default 0 is "NONE", already defined above
uint8 KEY_UP      = 1
uint8 KEY_DOWN    = 2
uint8 KEY_LEFT    = 3
uint8 KEY_RIGHT   = 4
uint8 KEY_ENTER   = 5
uint8 KEY_ESCAPE  = 6

