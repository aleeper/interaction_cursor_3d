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

    self.pub = rospy.Publisher("/rviz/interaction_cursor/update", InteractionCursorUpdate)
    self.sub = rospy.Subscriber("hydra_calib", Hydra, self.hydraCB)
    
    self.dragging = False
    #rate_float = 10
    #self.rate = rospy.Rate(rate_float)
    self.rate = rospy.Rate(10)
    print "Setup and waiting for hydra messages..."
    

  def hydraCB(self, msg):
    
    paddle = msg.paddles[0];

    icu = InteractionCursorUpdate() 
    icu.pose.pose.position.x = paddle.transform.translation.x + 1.0
    icu.pose.pose.position.y = paddle.transform.translation.y - 0.4
    icu.pose.pose.position.z = paddle.transform.translation.z * 1.25

    icu.pose.pose.orientation = paddle.transform.rotation
    icu.pose.header.frame_id = "torso_lift_link"
    
    #print "NONE = %d, KEEP_ALIVE = %d, GRAB = %d, RELEASE = %d"%(icu.NONE, icu.KEEP_ALIVE, icu.GRAB, icu.RELEASE)

    button = paddle.buttons[0]
    #print button
    if (button and not self.dragging):
      self.dragging = True
      icu.button_state = icu.GRAB
    elif (button and self.dragging):
      icu.button_state = icu.KEEP_ALIVE
    elif (not button and self.dragging):
      self.dragging = False
      icu.button_state = icu.RELEASE
    elif (not button and not self.dragging):
      icu.button_state = icu.NONE
    
    print "Publishing a message!" # button = %d"%(button)
    self.pub.publish(icu)
    #self.rate.sleep()

if __name__ == '__main__':
  rospy.init_node("interaction_cursor_test", anonymous = True)
  InteractionTest(rospy.get_name())
  rospy.spin()
