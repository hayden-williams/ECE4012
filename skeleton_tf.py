import rospy
import roslib
import tf
import geometry_msgs.msg

if __name__ == '__main__':
  rospy.init_node('listener_skeleton', anonymous=False)
  listener = tf.TransformListener()

  rate = rospy.Rate(10)
  while not rospu.is_shutdown():
    try:
      (trans,rot) = listener.lookupTransform ('/openni_depth_frame', 'torso_2', rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
      continue

    x = trnas[0]
    y = trans[1]
    z = trans[2]
    rate.sleep()