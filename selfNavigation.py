# Authors: Stephen Hayden Williams and Edgardo Marchand
# Date Created: 5 April 2018
# Date Revised: 12 April 2018

# This code should handel the basic self-navigation of the rover to the user
# Recieves rover bearing, direction and distance and travels to that location
# The direction and distance change for each leg of the journey
# Bearing changes as often as possible

# Initial code taken from our previous code: turninplace_userinput.py
# Issues needed fixing: 
#    <none>

# On TurtleBot:
# roslaunch turtlebot_bringup minimal.launch
# roslaunch openni_launch openni.launch
# On work station:
# python selfNavigation.py

import rospy
import roslib
from geometry_msgs.msg import Twist#, Pose
from nav_msgs.msg import Odometry
from cmath import *
from math import fabs
import requests
import time
import json
#from tf2_msgs.msg import TFMessage
#import tf

class selfNavigation():
	thetaError = 0
	kTurn = 1.5*pi/180

	direction = 1000
	bearing = 1000
	length = 0

	def __init__(self):
		# initiliaze
		rospy.init_node('selfNavigation', anonymous=False)

		# tell user how to stop TurtleBot
		rospy.loginfo("To stop TurtleBot CTRL + C")

		# What function to call when you ctrl + c    
		rospy.on_shutdown(self.shutdown)

		#rospy.Subscriber("/mobile_base/events/bumper",BumperEvent,self.BumperEventCallback)
		#rospy.Subscriber("/mobile_base/events/wheel_drop",WheelDropEvent,self.WheelDropEventCallback)
		#rospy.Subscriber('odom',Odometry,self.Orientation)

		# may need rospy.spin(); 

		
		# Create a publisher which can "talk" to TurtleBot and tell it to move
		# Tip: You may need to change cmd_vel_mux/input/navi to /cmd_vel if you're not using TurtleBot2
		self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
	 
		#TurtleBot will stop if we don't keep telling it to move.  How often should we tell it to move? 10 HZ
		r = rospy.Rate(20); #use to be 10

		# Twist is a datatype for velocity
		move_cmd = Twist()
		# let's go forward at 0.2 m/s
		move_cmd.linear.x = 0.0
		# let's turn at 0 radians/s
		move_cmd.angular.z = 0

		
		# as long as you haven't ctrl + c keeping doing...
		while not rospy.is_shutdown():

			# get info from server
			r = requests.get('http://128.61.7.199:3000/rover').json()
			print(r)

			self.direction = r['direction']
			self.length = r['length']
			self.bearing = r['bearing']
			emergency = r['emergency']
			end = r['ended'] # user ended trip
			arrived = r['arrived']
			home = r['gotHome'] # rover is home
			goToUser = r['goToUser']

			if goToUser==1:
				# put navigation code here
				# do error corrections
				print('entered goToUser')
				if self.direction != 1000:
					self.thetaError = self.direction - self.bearing # +ve = turn right, -ve = turn left 


				if (self.direction == 1000 or self.length == 0):
					move_cmd.linear.x = 0.0
					move_cmd.angular.z = 0
				elif fabs(self.thetaError) < 0.05:
					move_cmd.angular.z = self.kTurn*self.thetaError
					move_cmd.linear.x = 0.2
				elif fabs(self.thetaError) > 0.1:
					move_cmd.angular.z = self.kTurn*self.thetaError
					move_cmd.linear.x = 0.0
				else:
					move_cmd.angular.z = self.kTurn*self.thetaError
					move_cmd.linear.x = 0.0

				# publish the velocity
				self.cmd_vel.publish(move_cmd)


				# wait for 0.1 seconds (10 HZ) and publish again
				r.sleep()
			else:
				# do nothing
				move_cmd.linear.x = 0.0
				move_cmd.angular.z = 0
				self.cmd_vel.publish(move_cmd)

			


	def shutdown(self):
		# stop turtlebot
		rospy.loginfo("Stop TurtleBot")
		# a default Twist has linear.x of 0 and angular.z of 0.  So it'll stop TurtleBot
		self.cmd_vel.publish(Twist())
		# sleep just makes sure TurtleBot receives the stop command prior to shutting down the script
		rospy.sleep(1)
 
if __name__ == '__main__':
	try:
		selfNavigation()
	except:
		rospy.loginfo("selfNavigation node terminated.")
