import rospy
import roslib
import tf
import geometry_msgs.msg
from geometry_msgs.msg import Twist#, Pose

if __name__ == '__main__':

  num = 0
  desired = 10
  thetaError = 0
  kTurn = 5
  rospy.init_node('listener_skeleton', anonymous=False)
  listener = tf.TransformListener()
  cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
  move_cmd = Twist()
  # let's go forward at 0.2 m/s
  move_cmd.linear.x = 0.0
    # let's turn at 0 radians/s
  move_cmd.angular.z = 0

  rate = rospy.Rate(10)
  while not rospy.is_shutdown():
    torso = 'torso_' + str(num)
    print torso
    try:
      (trans,rot) = listener.lookupTransform('/openni_depth_frame', torso, rospy.Time(0) )
      x = trans[0]
      y = trans[1]
      z = trans[2]
      move_cmd.linear.x = 0.2
      cmd_vel.publish(move_cmd)

      print x
    except:
      num = num + 1
      if num == 10:
        num = 1
      print 'Looking for user'
      continue

    

    rate.sleep()