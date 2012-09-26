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
    #rate_float = 10
    #self.rate = rospy.Rate(rate_float)
    #rospy.Rate(2).sleep()
    print "Setup and waiting for hydra messages..."
    

  def hydraCB(self, msg):
    
    paddle = msg.paddles[0];

    icu = InteractionCursorUpdate() 
    icu.pose.pose.position.x = paddle.transform.translation.x
    icu.pose.pose.position.y = paddle.transform.translation.y
    icu.pose.pose.position.z = paddle.transform.translation.z

    icu.pose.pose.orientation = paddle.transform.rotation
    icu.pose.header.frame_id = "base_link"

    print "Publishing a message!" #, q.w = %0.2f"%(q.w)
    self.pub.publish(icu)

if __name__ == '__main__':
  rospy.init_node("interaction_cursor_test", anonymous = True)
  InteractionTest(rospy.get_name())
  rospy.spin()
