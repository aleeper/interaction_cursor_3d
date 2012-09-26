geometry_msgs/PoseStamped pose

uint8 button_state

# Type of button state:
# NONE: No buttons are pressed.
# KEEP_ALIVE: sent while dragging to keep up control of the marker.
# GRAB: Start dragging a marker (e.g. a "mouse_down" event).
# RELEASE: Stop dragging a marker (e.g. a "mouse_up" event)
# QUERY_MENU: Request menu options (e.g. a "right-click).
uint8 NONE = 0
uint8 KEEP_ALIVE = 1
uint8 GRAB = 2
uint8 RELEASE = 3
uint8 QUERY_MENU = 4


