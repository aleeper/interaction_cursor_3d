#!/usr/bin/env python

import roslib
roslib.load_manifest("interaction_cursor_rviz")

import rospy
from math import *
from interaction_cursor_msgs.msg import InteractionCursorUpdate
from razer_hydra.msg import Hydra, HydraPaddle
from geometry_msgs.msg import Point, Quaternion, Pose, PoseStamped



class InteractionTest(object):

  def __init__(self, name):

    self.pub = rospy.Publisher("/interaction_cursor_right/update", InteractionCursorUpdate)
    self.sub = rospy.Subscriber("hydra_calib", Hydra, self.hydraCB)
    
    self.left_down = False
    self.right_down = False
    self.joy_click = False
    self.joystick_active = False
    #rate_float = 10
    #self.rate = rospy.Rate(rate_float)
    self.rate = rospy.Rate(10)
    print "Setup and waiting for hydra messages..."
    

  def hydraCB(self, msg):
    
    paddle = msg.paddles[msg.RIGHT];

    icu = InteractionCursorUpdate() 
    icu.pose.pose.position.x = paddle.transform.translation.x + 1.0
    icu.pose.pose.position.y = paddle.transform.translation.y - 0.4
    icu.pose.pose.position.z = paddle.transform.translation.z * 1.25

    icu.pose.pose.orientation = paddle.transform.rotation
    icu.pose.header.frame_id = "torso_lift_link"
    
    #print "NONE = %d, KEEP_ALIVE = %d, GRAB = %d, RELEASE = %d"%(icu.NONE, icu.KEEP_ALIVE, icu.GRAB, icu.RELEASE)

    button = paddle.trigger > 0.8
    right_click = paddle.buttons[0]
    if (button and not self.left_down):
      self.left_down = True
      icu.button_state = icu.GRAB
    elif (button and self.left_down):
      icu.button_state = icu.KEEP_ALIVE
    elif (not button and self.left_down):
      self.left_down = False
      icu.button_state = icu.RELEASE
    elif (not button and not self.left_down):
      icu.button_state = icu.NONE
      self.left_down = False
    
    if (right_click and not self.right_down):
      self.right_down = True
      icu.button_state = icu.QUERY_MENU
    elif (right_click and self.right_down):
      pass
    elif (not right_click and self.right_down):
      self.right_down = False
    elif (not right_click and not self.right_down):
      self.right_down = False
   
    joy = paddle.joy
    joy_on = abs(joy[0]) > 0.8 or abs(joy[1]) > 0.8
    if (joy_on and not self.joystick_active):
      self.joystick_active = True
      icu.key_event = (joy[0]>0.8)*icu.KEY_RIGHT + (joy[0]<-0.8)*icu.KEY_LEFT + (joy[1]>0.8)*icu.KEY_UP + (joy[1]<-0.8)*icu.KEY_DOWN
    elif (joy_on and self.joystick_active):
      pass
    elif (not joy_on and self.joystick_active):
      self.joystick_active = False
    elif (not joy_on and not self.joystick_active):
      self.joystick_active = False
    
    joy_click = paddle.buttons[6]
    if (joy_click and not self.joy_click):
      self.joy_click = True
      icu.key_event = icu.KEY_ENTER
    elif (joy_click and self.joy_click):
      pass
    elif (not joy_click and self.joy_click):
      self.joy_click = False
    elif (not joy_click and not self.joy_click):
      self.joy_click = False
     
    print "Publishing a message! Button: %d, Key: %d"%(icu.button_state, icu.key_event) # button = %d"%(button)
    self.pub.publish(icu)
    #self.rate.sleep()

if __name__ == '__main__':
  rospy.init_node("interaction_cursor_test", anonymous = True)
  InteractionTest(rospy.get_name())
  rospy.spin()
