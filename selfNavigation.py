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
from math import *
import requests
import time
import json
#from tf2_msgs.msg import TFMessage
#import tf
import sys 
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

class selfNavigation():
	thetaError = 0
	kTurn = 1.5

	direction = 1000
	bearing = 1000
	length = 0
	countQuery = 0

	odomBearing = 0
	zeroAngle = 1000
	desiredAngle = 0

	xstart = 0
	ystart = 0
	magnitude = 9999999.0

	z_thresh = 1000
	z_threshCorner = z_thresh
	ZoneList = np.array([0,0,0,0,0,0])
	count = 0

	path = 0
	x = 0
	y = 0
	

	def __init__(self):
		# initiliaze
		rospy.init_node('selfNavigation', anonymous=False)

		# tell user how to stop TurtleBot
		rospy.loginfo("To stop TurtleBot CTRL + C")

		# What function to call when you ctrl + c    
		rospy.on_shutdown(self.shutdown)

		#rospy.Subscriber("/mobile_base/events/bumper",BumperEvent,self.BumperEventCallback)
		#rospy.Subscriber("/mobile_base/events/wheel_drop",WheelDropEvent,self.WheelDropEventCallback)
		rospy.Subscriber('odom',Odometry,self.Orientation)

		# may need rospy.spin(); 

		# Initialize bridge
		self.bridge = CvBridge()
		# Subscribe to depth sensor and get raw image
		self.depth_sub = rospy.Subscriber("/camera/depth/image_raw",Image,self.callback)

		
		# Create a publisher which can "talk" to TurtleBot and tell it to move
		# Tip: You may need to change cmd_vel_mux/input/navi to /cmd_vel if you're not using TurtleBot2
		self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
	 
		#TurtleBot will stop if we don't keep telling it to move.  How often should we tell it to move? 10 HZ
		self.r = rospy.Rate(10) #use to be 10

		# Twist is a datatype for velocity
		move_cmd = Twist()
		# let's go forward at 0.2 m/s
		move_cmd.linear.x = 0.0
		# let's turn at 0 radians/s
		move_cmd.angular.z = 0

		
		# as long as you haven't ctrl + c keeping doing...
		while not rospy.is_shutdown():

			# get info from server
			if self.countQuery%1000 == 0:
				#only check server occationally
				re = requests.get('http://128.61.7.199:3000/rover').json()
				rospy.loginfo(re)

				self.direction = re['direction'] # in degrees
				self.length = re['len']
				self.bearing = re['bearing']
				emergency = re['emergency']
				end = re['ended'] # user ended trip
				arrived = re['arrived']
				home = re['gotHome'] # rover is home
				goToUser = re['goToUser']


			self.countQuery = self.countQuery + 1

			if goToUser==1:
				# put navigation code here
				# do error corrections
				rospy.loginfo('entered goToUser')
				
				self.desiredAngle = (360-self.direction[self.path])*3.14159265359/180 # input degree convert to rad
				# using odometry for bearing
				# IndoorAtlus East is 90, Odometry West is 90, Need to account for this
				
				

				rospy.loginfo(np.absolute(self.thetaError))
				if (np.absolute(self.thetaError) < 0.25):
					self.count = 0
			
				rospy.loginfo("obstacle " + str(self.ZoneList))
				if (np.sum(self.ZoneList) == 0):

					#no obstacle, move code goes here
					rospy.loginfo("after desiredAngle")
					rospy.loginfo(self.magnitude)
					rospy.loginfo(self.length)
					rospy.loginfo(self.direction)
					if (self.direction[self.path] == 1000 or self.length[self.path] == 0.0 or self.zeroAngle == 1000 or self.magnitude == 9999999.0):
						rospy.loginfo('len = 0, dir = 1000')
						move_cmd.linear.x = 0.0
						move_cmd.angular.z = 0
					elif (fabs(self.thetaError) < 1 and self.magnitude < self.length[self.path]):
						rospy.loginfo('error < 0.05')
						move_cmd.angular.z = self.kTurn*self.thetaError
						move_cmd.linear.x = 0.2
					elif (fabs(self.thetaError) > 1 and self.magnitude < self.length[self.path]):
						rospy.loginfo('error>0.05')
						move_cmd.angular.z = self.kTurn*self.thetaError
						move_cmd.linear.x = 0.0
					elif (self.magnitude >= self.length[self.path]):
						rospy.loginfo('error>0.05')
						move_cmd.angular.z = 0.0
						move_cmd.linear.x = 0.0
						self.path = self.path + 1
						self.xstart = self.x + self.xstart
						self.ystart = self.y + self.ystart
						self.magnitude = 0

					else:
						rospy.loginfo('else')
						move_cmd.angular.z = self.kTurn*self.thetaError
						move_cmd.linear.x = 0.0


				else:
					if (self.ZoneList[0] == 0 and self.ZoneList[1] == 0 and self.ZoneList[2] == 0 and self.ZoneList[3] != 0):
						#rospy.loginfo("inside else")
						#soft left
						move_cmd.linear.x = 0.2
						move_cmd.angular.z = 0.5
					elif (self.ZoneList[0] != 0 and self.ZoneList[1] == 0 and self.ZoneList[2] == 0 and self.ZoneList[3] == 0):
						#soft right
						move_cmd.linear.x = 0.2
						move_cmd.angular.z = -0.5
					elif (self.ZoneList[0] == 0 and self.ZoneList[1] != 0 and self.ZoneList[2] != 0 and self.ZoneList[3] == 0):
						if (self.ZoneList[1] > self.ZoneList[2]):
							#Hard Right
							move_cmd.linear.x = 0.2
							move_cmd.angular.z = -0.75
						else:
							move_cmd.linear.x = 0.2
							move_cmd.angular.z = 0.7
							#Hard Leff
					elif((self.ZoneList[0]==0 and self.ZoneList[2] !=0) or (self.ZoneList[0] == 0 and self.ZoneList[1] !=0 and self.ZoneList[3] != 0)):
						move_cmd.linear.x = 0.2
						move_cmd.angular.z = 0.7
					# Hard Left
					elif((self.ZoneList[1] != 0 and self.ZoneList[3] ==0) or (self.ZoneList[0] != 0 and self.ZoneList[2] != 0 and self.ZoneList[3] == 0)):
						move_cmd.linear.x = 0.2
						move_cmd.angular.z = -0.75
					else:
						self.count = self.count + 1
						if self.count == 1 :
							move_cmd.linear.x = 0.0
							move_cmd.angular.z = 0
							self.cmd_vel.publish(move_cmd)
							self.r.sleep()
							while (np.sum(self.ZoneList) != 0 and np.absolute(self.thetaError) < 1.57):
								move_cmd.angular.z = 0.5
								self.cmd_vel.publish(move_cmd)
								self.r.sleep()
						elif self.count == 2 :
							move_cmd.linear.x = 0.0
							move_cmd.angular.z = 0
							self.cmd_vel.publish(move_cmd)
							self.r.sleep()
							while (np.sum(self.ZoneList)!=0):
								move_cmd.angular.z = -0.5
								self.cmd_vel.publish(move_cmd)
								self.r.sleep()
						else:
							rospy.loginfo("I cant make it around! Help Mommy")
							move_cmd.linear.x = 0.0
							move_cmd.angular.z = 0
							self.cmd_vel.publish(move_cmd)


				# publish the velocity
				rospy.loginfo('publish')
				self.cmd_vel.publish(move_cmd)


				# wait for 0.1 seconds (10 HZ) and publish again
				#r.sleep()
			else:
				# do nothing
				rospy.loginfo('do nothing')
				move_cmd.linear.x = 0.0
				move_cmd.angular.z = 0
				self.cmd_vel.publish(move_cmd)

	def Orientation(self,data):
		qz = data.pose.pose.orientation.z
		qw = data.pose.pose.orientation.w
		current = qw + qz*1j
		if self.zeroAngle == 1000:
			self.zeroAngle = (qw + qz*1j)**2
			self.xstart = data.pose.pose.position.x
			self.ystart = data.pose.pose.position.y 
			# at this point, zeroAngle is our 0
			#self.zeroAngle = self.zeroAngle*(cos(self.desiredAngle)+sin(self.desiredAngle)*1j)
		else:
			Angle = self.zeroAngle*(cos(self.desiredAngle)+sin(self.desiredAngle)*1j)
			error = Angle/(current**2)
			self.thetaError = phase(error) # radians from 0, -pi to pi	

		self.x = data.pose.pose.position.x - self.xstart
		self.y = data.pose.pose.position.y - self.ystart
		self.magnitude = sqrt(x**2 + y**2)

	def callback(self,data):
		try:
			


			# Get Image and find size of image
			self.depth_image = self.bridge.imgmsg_to_cv2(data, "passthrough")
			rows, col, channels = self.depth_image.shape #grey scale channel is 1, rgb is 3

			# Find Center of Image
			cR = np.int(np.round(rows/2))
			cC = np.int(np.round(col/2))


			colFrac = np.int(np.round(.25*col))


			self.mask =  np.zeros((rows,col))
			self.mask[cR-rows*.25:rows,0:col] = 5
			self.mask = np.uint16(self.mask)
			self.mask = cv2.inRange(self.mask,np.array(4,dtype = "uint16"),np.array(6,dtype = "uint16"))


			min_z= np.array(100, dtype = "uint16") #bgr
			max_z= np.array(self.z_thresh, dtype = "uint16")
			self.mask2 = cv2.inRange(self.depth_image, min_z, max_z)
			
			#Combination of masks
			self.mask3 = cv2.bitwise_and(self.mask,self.mask, mask= self.mask2)

			min_zCorn= np.array(100, dtype = "uint16") #bgr
			max_zCorn= np.array(self.z_threshCorner, dtype = "uint16")
			self.maskCorner = cv2.inRange(self.depth_image, min_zCorn, max_zCorn)

			self.maskZone1 = np.zeros((rows,col))
			self.maskZone1[0:rows,0:np.round(col/4)] = 5
			self.maskZone1 = np.uint16(self.maskZone1)
			self.maskZone1 = cv2.inRange(self.maskZone1,np.array(4,dtype = "uint16"),np.array(6,dtype = "uint16"))
			self.Zone1 = cv2.bitwise_and(self.maskCorner,self.maskCorner, mask= self.maskZone1)

			self.maskZone2 = np.zeros((rows,col))
			self.maskZone2[0:rows,np.round(col/4)+1:np.round(col/2)] = 5
			self.maskZone2 = np.uint16(self.maskZone2)
			self.maskZone2 = cv2.inRange(self.maskZone2,np.array(4,dtype = "uint16"),np.array(6,dtype = "uint16"))
			self.Zone2 = cv2.bitwise_and(self.mask3,self.mask3, mask= self.maskZone2)

			self.maskZone3 = np.zeros((rows,col))
			self.maskZone3[0:rows,np.round(col/2)+1:cC+np.round(col/4)] = 5
			self.maskZone3 = np.uint16(self.maskZone3)
			self.maskZone3 = cv2.inRange(self.maskZone3,np.array(4,dtype = "uint16"),np.array(6,dtype = "uint16"))
			self.Zone3 = cv2.bitwise_and(self.mask3,self.mask3, mask= self.maskZone3)

			self.maskZone4 = np.zeros((rows,col))
			self.maskZone4[0:rows,cC+np.round(col/4)+1:col] = 5
			self.maskZone4 = np.uint16(self.maskZone4)
			self.maskZone4 = cv2.inRange(self.maskZone4,np.array(4,dtype = "uint16"),np.array(6,dtype = "uint16"))
			self.Zone4 = cv2.bitwise_and(self.mask3,self.mask3, mask= self.maskZone4)


			sumZone1 = np.sum(self.Zone1 / 255)
			#rospy.loginfo("sum of Zone1 is " + str(sumZone1))
			sumZone2 = np.sum(self.Zone2 / 255)
			#rospy.loginfo("sum of Zone2 is " + str(sumZone2))
			sumZone3 = np.sum(self.Zone3 / 255)
			#rospy.loginfo("sum of Zone3 is " + str(sumZone3))
			sumZone4 = np.sum(self.Zone4 / 255)
			#rospy.loginfo("sum of Zone4 is " + str(sumZone4))


			self.ZoneList = np.array([sumZone1, sumZone2, sumZone3, sumZone4])
			#rospy.loginfo("Zone List is "+ str(self.ZoneList))




				


			self.r.sleep()


		except CvBridgeError, e:
			print e

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
