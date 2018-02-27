#!/usr/bin/env python  
import roslib
import rospy
import math
import tf
import geometry_msgs.msg
    
if __name__ == '__main__':
   rospy.init_node('turtle_tf_listener')

   listener = tf.TransformListener()



   rate = rospy.Rate(10.0)
   while not rospy.is_shutdown():
      try:
          (trans,rot) = listener.lookupTransform('/torso_2', '/openni_depth_frame', rospy.Time(0))
      except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
          continue

      print trans[1]
      print "test"



      rate.sleep()