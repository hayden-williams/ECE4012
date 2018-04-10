# Authors: Stephen Hayden Williams and Edgardo Marchand
# Date Created: 18 Oct 2017
# Date Revised: 18 Oct 2017

# A very basic TurtleBot script that moves TurtleBot forward, bumper paused the movement for 2 sec. Press CTRL + C to stop.  To run:
# On TurtleBot:
# roslaunch turtlebot_bringup minimal.launch
# On work station:
# python goStraight.py

import rospy
import sys 
import roslib
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from geometry_msgs.msg import Twist#, Pose
from nav_msgs.msg import Odometry
from cmath import *
#from obstacle_detect import obstacle_detect
#from tf2_msgs.msg import TFMessage
#import tf

class GoStraight():
	desired = 10 # should never naturally be 10, this was to give bot time to get correct error
	thetaError = 0
	kTurn = 5
	#obstacle = obstacle_detect()

	#Threshold for detecting object in a zone
	#Zones go from left to right on image 
	# Distance in mm
	z_thresh = 1000
	z_threshCorner = z_thresh
	ZoneList = np.array([0,0,0,0,0,0])

	def __init__(self):
		# initiliaze
		rospy.init_node('GoStraight', anonymous=False)
		#obs = obstacle_detect()

		# tell user how to stop TurtleBot
		rospy.loginfo("To stop TurtleBot CTRL + C")

		# What function to call when you ctrl + c    
		rospy.on_shutdown(self.shutdown)

		#rospy.Subscriber("/mobile_base/events/bumper",BumperEvent,self.BumperEventCallback)
		#rospy.Subscriber("/mobile_base/events/wheel_drop",WheelDropEvent,self.WheelDropEventCallback)
		rospy.Subscriber('odom',Odometry,self.Orientation)

		# Initialize bridge
		self.bridge = CvBridge()
		# Subscribe to depth sensor and get raw image
		self.depth_sub = rospy.Subscriber("/camera/depth/image_raw",Image,self.callback)
		#self.depth_sub = rospy.Subscriber("/camera/depth_registered/image_raw",Image,self.callback_depth)
		# Publish to navigation to move robot

		# may need rospy.spin(); 

		
		# Create a publisher which can "talk" to TurtleBot and tell it to move
		# Tip: You may need to change cmd_vel_mux/input/navi to /cmd_vel if you're not using TurtleBot2
		self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
	 
		#TurtleBot will stop if we don't keep telling it to move.  How often should we tell it to move? 10 HZ
		self.r = rospy.Rate(10);

		# Twist is a datatype for velocity
		move_cmd = Twist()
		# let's go forward at 0.2 m/s
		move_cmd.linear.x = 0.0
		# let's turn at 0 radians/s
		move_cmd.angular.z = 0

		
		# as long as you haven't ctrl + c keeping doing...
		while not rospy.is_shutdown():
		
			rospy.loginfo("obstacle " + str(self.ZoneList))
			if (np.sum(self.ZoneList) == 0):

				if self.desired == 10:
					move_cmd.linear.x = 0.0
					move_cmd.angular.z = 0
				else:
					move_cmd.linear.x = 0.2
					move_cmd.angular.z = self.kTurn*self.thetaError
			else:
				rospy.loginfo("inside else")
				move_cmd.linear.x = 0.1
				move_cmd.angular.z = 1


			# publish the velocity
			self.cmd_vel.publish(move_cmd)
			# wait for 0.1 seconds (10 HZ) and publish again
			self.r.sleep()

	def Orientation(self,data):
		qz = data.pose.pose.orientation.z
		qw = data.pose.pose.orientation.w
		current = qw + qz*1j
		if self.desired == 10:
			self.desired = (qw + qz*1j)**2
		else:
			error = self.desired/(current**2)
			self.thetaError = phase(error)
		#desired = 1 + 0*1j
		#thetaDesired = 0
		#desired = cos(thetaDesired)+sin(thetaDesired)*1j
		#error = desired/current
		#thetaError = phase(error)
		#rospy.loginfo("qz: %f qw: %f"%(qz, qw))
		
		#thetaZ = qz/sqrt(1-(qw*qw))
		#euler = self.tf.transformations.euler_from_quaternion(quaternion)
		#yaw = euler[2]
		
		#rospy.loginfo("theta = %f"%(self.thetaError))

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
		GoStraight()

	except:
		rospy.loginfo("GoStraight node terminated.")
