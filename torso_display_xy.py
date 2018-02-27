#!/usr/bin/env python  
import roslib
import rospy
import math
import tf
import geometry_msgs
import sys
    

   
class torso_display_xy():

  def __init__(self):
    self.listener = tf.TransformListener()



    self.rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
      try:
        (trans,rot) = self.listener.lookupTransform('/torso_2', '/openni_depth_frame', rospy.Time(0))
      except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        continue

      print trans[1]
      print "test"



      rate.sleep()


def main(args):
  torso = torso_display_xy()
  rospy.init_node('turtle_tf_listener')
  try:

    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"

if __name__ == '__main__':
    main(sys.argv)