

# A very basic TurtleBot script that moves TurtleBot forward, bumper paused the movement for 2 sec. Press CTRL + C to stop.  To run:
# On TurtleBot:
# roslaunch turtlebot_bringup minimal.launch
# On work station:
# python goforwardWithBumper.py

import rospy
import roslib
import tf
import geometry_msgs.msg

#from geometry_msgs.msg import Pose
class GoForward():
  stateMachine = 0
  counter = 0
  num = 0
  exist = 0

  def __init__(self):
    # initiliaze
    rospy.init_node('GoForward', anonymous=False)

    # tell user how to stop TurtleBot
    rospy.loginfo("To stop TurtleBot CTRL + C")

    # What function to call when you ctrl + c    
    rospy.on_shutdown(self.shutdown)
    # Create a publisher which can "talk" to TurtleBot and tell it to move
    # Tip: You may need to change cmd_vel_mux/input/navi to /cmd_vel if you're not using TurtleBot2
    self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
   
    #TurtleBot will stop if we don't keep telling it to move.  How often should we tell it to move? 10 HZ
    r = rospy.Rate(10);

    # Twist is a datatype for velocity
    move_cmd = Twist()
    # let's go forward at 0.2 m/s
    move_cmd.linear.x = 0
    # let's turn at 0 radians/s
    move_cmd.angular.z = 0

    # Listenr for Joints
    #listener = tf.TransformLister()



    
    # as long as you haven't ctrl + c keeping doing...
    while not rospy.is_shutdown():
    
      #rospy.loginfo("stateMachine %d."%(self.stateMachine))
      print "Hello from while not rospy.is_shutdown"
      while self.exist < 1:
        try:
          (trans,rot) = self.listener.lookupTransform('/openni_depth_frame', 'torso_%f'%(self.num), rospy.Time(0) )
          self.exist = 1
        except:
          self.num = self.num + 1
          if self.num == 10:
            self.num = 1
          print 'failed'
          continue
      print trans
      self.cmd_vel.publish(move_cmd)
      # wait for 0.1 seconds (10 HZ) and publish again
      r.sleep()


      
   
    
  def shutdown(self):
    # stop turtlebot
    rospy.loginfo("Stop TurtleBot")
    # a default Twist has linear.x of 0 and angular.z of 0.  So it'll stop TurtleBot
    self.cmd_vel.publish(Twist())
    # sleep just makes sure TurtleBot receives the stop command prior to shutting down the script
    rospy.sleep(1)
 
if __name__ == '__main__':
  try:
    GoForward()
  except:
    rospy.loginfo("GoForward node terminated.")
