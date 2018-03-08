import rospy
import roslib
import tf
import geometry_msgs.msg

if __name__ == '__main__':

  num = 2
  rospy.init_node('listener_skeleton', anonymous=False)
  listener = tf.TransformListener()

  rate = rospy.Rate(10)
  while not rospy.is_shutdown():
    torso = "torso_" + str(num)
    try:
      (trans,rot) = listener.lookupTransform ('/openni_depth_frame', torso, rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
      continue

    x = trans[0]
    y = trans[1]
    z = trans[2]
    print x
    rate.sleep()