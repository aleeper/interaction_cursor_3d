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

uint8 key_event
# default 0 is "NONE"
uint8 KEY_UP = 1
uint8 KEY_DOWN = 2
uint8 KEY_LEFT = 3
uint8 KEY_RIGHT = 4
uint8 KEY_ENTER = 5
