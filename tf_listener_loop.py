import rospy
import roslib
import tf
import geometry_msgs.msg

if __name__ == '__main__':

  num = 0
  rospy.init_node('listener_skeleton', anonymous=False)
  listener = tf.TransformListener()

  rate = rospy.Rate(10)
  while not rospy.is_shutdown():
    torso = 'torso_' + str(self.num)
    print torso
    try:
      (trans,rot) = self.listener.lookupTransform('/openni_depth_frame', torso, rospy.Time(0) )

      print trans
    except:
      self.num = self.num + 1
      if self.num == 10:
        self.num = 1
      print 'Looking for user'
      continue

    x = trans[0]
    y = trans[1]
    z = trans[2]
    print x
    rate.sleep()