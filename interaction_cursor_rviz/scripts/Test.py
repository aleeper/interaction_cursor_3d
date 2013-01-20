#!/usr/bin/env python

import roslib
roslib.load_manifest("interaction_cursor_rviz")

import rospy
from math import *
from interaction_cursor_msgs.msg import InteractionCursorUpdate
from geometry_msgs.msg import Point, Quaternion, Pose, PoseStamped

rospy.init_node("interaction_cursor_test", anonymous = True)
pub = rospy.Publisher("/rviz/interaction_cursor/update", InteractionCursorUpdate)

rate_float = 10
rate = rospy.Rate(rate_float)
rospy.Rate(2).sleep()

while not rospy.is_shutdown():
  t = rospy.get_time()
  icu = InteractionCursorUpdate()
  
  p = Point(0,-3*(1+sin(0.1*t*(2*pi))),0)
  q = Quaternion(0,0,0,1.0)
  icu.pose.pose.position = p
  icu.pose.pose.orientation = q
  icu.pose.header.frame_id = "base_link"

  print("Publishing a message!") #, q.w = %0.2f"%(q.w)
  pub.publish(icu)
  #print "Sleeping..."
  rate.sleep()
  #print "End of loop!"

