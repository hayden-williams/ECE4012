# Created by Edgardo Marchand
# Created 3/30/2018
# Senior Design Spring 2018 Georgia Institute of Technology

# This is a basic follower code that allows a trutlebot with kinect to follow an object / person at a certain distance.
# It was designed to work in Georgia Tech's Van Leer Building but can be easily adjusted by changing threshold parameters
# The main idea is:
# Find an object in a specific quadrant of the depth image produced by the kinect.
# It sets boundaries where it is looking at the top center half of the depth image to get the torso of a human
# It then masks that quadrant unto the image to only get the depth values in that image.
# It masks the resulting image with a lower and upper threshold for the depth values since this is the area the user will live in.
# It create a third mask that combines both masks previosly mention to exctract the depth values of our user. This correspond to how far the user is from the robot
# It then cleans up and creates an image with only the desired values.
# It then calculates the centroid of our user based on the image using the combined mask. This is usued to rotate the turtlebot to always face the user
# It also calculates the median of the depth values of the user and uses that to get our depth measurement of our user (can also use mean)
# The movement side is very simple. It looks at the distance of the user from the turlebot and assignes the appropiate speed to maintain a desired distance.

# Note Kinect outputs depth in Uint16

import rospy
import roslib; roslib.load_manifest('sound_play')
from geometry_msgs.msg import Twist#, Pose
from nav_msgs.msg import Odometry
from cmath import *
from math import *
import requests
import time
import json
import sys 
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from sound_play.libsoundplay import SoundClient

class following_final2():
	# Distance in mm
	thetaError = 0
	kTurn = 1.25

	direction = np.array([0,0,0,0,0])
	bearing = 1000
	length = np.array([0,0,0,0,0])
	countQuery = 0
	arrived = 0
	goToUser = 0
	emergency = 0
	goHome = 0
	end = 0

	odomBearing = 0
	zeroAngle = 1000
	desiredAngle = 0
 	xstart = 0
	ystart = 0
	magnitude = 9999999.0

	z_thresh = 800
	z_threshCorner = z_thresh
	ZoneList = np.array([0,0,0,0,0,0])
	count = 0

	path = 0
	x = 0
	y = 0

	  # Distance in mm
	invalid_thresh = 300
	desired_thresh = 1000
	desired_lowBound = 950
	desired_upBound = 1050
	invalid_max = 2000
	max_speed = 1
	soundCounter = 0

	savePic = 0
	emergencyFirstTime = 0

	roverAtUser = 0

	directionHolder = 1000
	lengthHolder = 0


	def __init__(self):
		rospy.init_node('image_converter', anonymous=True)

		rospy.loginfo("To stop TurtleBot CTRL + C")
		# Initialize bridge
		self.bridge = CvBridge()
		# Subscribe to depth sensor and get raw image
		self.depth_sub = rospy.Subscriber("/camera/depth/image_raw",Image,self.callback)
		#self.depth_sub = rospy.Subscriber("/camera/depth_registered/image_raw",Image,self.callback_depth)
		# Publish to navigation to move robot
		self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
		# Initialize Twist that will be published to cmd_vel
		self.move_cmd = Twist()

		self.r = rospy.Rate(10)

		rospy.on_shutdown(self.shutdown)

		self.image_sub = rospy.Subscriber("/camera/rgb/image_color",Image,self.callbackImage)

		rospy.Subscriber('odom',Odometry,self.Orientation)

		self.soundhandle = SoundClient()
		self.move_cmd.linear.x = 0.0
		# let's turn at 0 radians/s
		self.move_cmd.angular.z = 0


	def callback(self,data):
		try:
			#rospy.loginfo('Callback')
			"""
			self.countQuery = self.countQuery + 1
			if (self.countQuery == 10):
				rospy.loginfo('requesting stuff')
				re = requests.get('http://128.61.14.57:3000/rover').json()  #<-------------------SERVER IP ADDRESS HERE------------
				#rospy.loginfo(re)

				self.direction = re['direction'] # in degrees
				self.length = re['len']
				#self.bearing = re['bearing']
				self.emergency = re['emergency']
				self.end = re['ended'] # user ended trip
				self.arrived = re['arrived']
				#home = re['gotHome'] # rover is home
				self.goToUser = re['goToUser']
				#self.goHome = re['goHome']
				self.countQuery = 0
			"""
			if (self.goToUser == 1 or self.end == 1):
				rospy.loginfo('Autonomous')
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

				sumZone1 = np.sum(self.mask3[0:rows,0:np.round(col/4)] / 255)
				#rospy.loginfo("sum of Zone1 is " + str(sumZone1))
				sumZone2 = np.sum(self.mask3[0:rows,np.round(col/4)+1:np.round(col/2)] / 255)
				#rospy.loginfo("sum of Zone2 is " + str(sumZone2))
				sumZone3 = np.sum(self.mask3[0:rows,np.round(col/2)+1:cC+np.round(col/4)] / 255)
				#rospy.loginfo("sum of Zone3 is " + str(sumZone3))
				sumZone4 = np.sum(self.mask3[0:rows,cC+np.round(col/4)+1:col] / 255)
				#rospy.loginfo("sum of Zone4 is " + str(sumZone4))


				self.ZoneList = np.array([sumZone1, sumZone2, sumZone3, sumZone4])

				rospy.loginfo(self.magnitude)

				if (np.absolute(self.thetaError) < 0.3):
					self.count = 0

				if (np.sum(self.ZoneList) == 0 or self.magnitude < 0.15):

					if (self.path < 4):
						self.desiredAngle = (360-self.direction[self.path])*3.1416/180
					if (self.path > 4):
						#Rover at user
						rospy.loginfo('end of paths')
						self.move_cmd.angular.z = 0.0
						self.move_cmd.linear.x = 0.0
						self.roverAtUser = 1
						if (self.end == 1):
							tellServer = requests.post('http://128.61.14.57:3000/home', {'gotHome': 1})

					elif (fabs(self.thetaError) < 1.65 and self.magnitude <= self.length[self.path] ):
						self.move_cmd.angular.z = self.kTurn*self.thetaError
						self.move_cmd.linear.x = 0.2
					elif (fabs(self.thetaError) > 1.65 and self.magnitude <= self.length[self.path]):
						self.move_cmd.angular.z = self.kTurn*self.thetaError
						self.move_cmd.linear.x = 0.0
					elif (self.magnitude >= self.length[self.path]):
						self.move_cmd.angular.z = 0.0
						self.move_cmd.linear.x = 0.0
						self.path = self.path + 1
						self.xstart = self.x + self.xstart
						self.ystart = self.y + self.ystart
						self.magnitude = 0
				elif (np.sum(self.ZoneList) != 0 and self.roverAtUser == 0 and self.thetaError < 0.6):
					if (self.ZoneList[0] == 0 and self.ZoneList[1] == 0 and self.ZoneList[2] == 0 and self.ZoneList[3] != 0):
						self.move_cmd.linear.x = 0.2
						self.move_cmd.angular.z = 0.5
					elif (self.ZoneList[0] != 0 and self.ZoneList[1] == 0 and self.ZoneList[2] == 0 and self.ZoneList[3] == 0):
						#soft right
						self.move_cmd.linear.x = 0.2
						self.move_cmd.angular.z = -0.5
					elif (self.ZoneList[0] == 0 and self.ZoneList[1] != 0 and self.ZoneList[2] != 0 and self.ZoneList[3] == 0):
						if (self.ZoneList[1] > self.ZoneList[2]):
						  #Hard Right
						  self.move_cmd.linear.x = 0.2
						  self.move_cmd.angular.z = -0.75
						else:
						  self.move_cmd.linear.x = 0.2
						  self.move_cmd.angular.z = 0.7
						  #Hard Leff
					elif((self.ZoneList[0]==0 and self.ZoneList[2] !=0) or (self.ZoneList[0] == 0 and self.ZoneList[1] !=0 and self.ZoneList[3] != 0)):
						self.move_cmd.linear.x = 0.2
						self.move_cmd.angular.z = 0.7
					  # Hard Left
					elif((self.ZoneList[1] != 0 and self.ZoneList[3] ==0) or (self.ZoneList[0] != 0 and self.ZoneList[2] != 0 and self.ZoneList[3] == 0)):
						self.move_cmd.linear.x = 0.2
						self.move_cmd.angular.z = -0.75
					elif (self.ZoneList[0] != 0 and self.ZoneList[3] != 0 and self.count<1):
						while (np.absolute(self.thetaError) < 1.57):
							self.move_cmd.angular.z = 0.3
							self.move_cmd.linear.x = 0
							self.cmd_vel.publish(self.move_cmd)
							self.r.sleep()
						self.count = self.count + 1
					elif (self.ZoneList[0] != 0 and self.ZoneList[3] != 0 and self.count<2):
						while (np.absolute(self.thetaError) < 1.57):
							self.move_cmd.angular.z = -0.3
							self.move_cmd.linear.x = 0
							self.cmd_vel.publish(self.move_cmd)
							self.r.sleep()
						self.count = self.count +1
					elif(self.count == 2):
						rospy.loginfo("Im stuck")
						self.move_cmd.angular.z = 0.0
						self.move_cmd.linear.x = 0.0


				else:
					self.move_cmd.linear.x = 0.0
					self.move_cmd.angular.z = 0

				self.cmd_vel.publish(self.move_cmd)
				self.r.sleep()
			
			elif self.emergency == 1:
				# say emergency, do nothing except send pics
				rospy.loginfo('emergency')
				
				self.move_cmd.linear.x = 0.0
				self.move_cmd.angular.z = 0
				self.cmd_vel.publish(self.move_cmd)
				self.r.sleep()
				self.soundhandle.say('Emergency')
				if self.savePic == 0: self.savePic = 1
				rospy.sleep(2)

			elif (self.arrived == 1):
			# Gain Values for movement
				rospy.loginfo('Tracking Mode')
			# X gain rotation
				K = 0.0035
				# Kx is for movment in z direction forward backwards
				Kx = .0005

				# Get Image and find size of image
				self.depth_image = self.bridge.imgmsg_to_cv2(data, "passthrough")
				rows, col, channels = self.depth_image.shape #grey scale channel is 1, rgb is 3

				# Find Center of Image
				cR = np.int(np.round(rows/2))
				cC = np.int(np.round(col/2))

				# How Large our field of view for our desired object i.e where we looking in the picture for our object
				# Mask to remove any uneccesary information outside our looking area.
				rowFrac = np.int(np.round(.25*cR))
				colFrac = np.int(np.round(.4*cC))

				#Masks the quadrant it is interested on (default = upper center)
				self.mask2 =  np.zeros((rows,col))
				self.mask2[cR-(2*rowFrac):cR-rowFrac,cC-colFrac:cC+colFrac] = 5
				self.mask2 = np.uint16(self.mask2)
				self.mask2 = cv2.inRange(self.mask2,np.array(4,dtype = "uint16"),np.array(6,dtype = "uint16"))

				# Mask to get values of specific box in z direction only interested in our object/person
				min_z= np.array(500, dtype = "uint16") #bgr
				max_z= np.array(2000, dtype = "uint16")
				self.mask = cv2.inRange(self.depth_image, min_z, max_z)
				
				#Combination of masks
				self.mask3 = cv2.bitwise_and(self.mask,self.mask, mask= self.mask2)
				image = cv2.bitwise_and(self.depth_image,self.depth_image, mask= self.mask3)

				#Remove unnecesarry values
				image = image.astype(float)
				image[image==0] = np.nan
				image = image[~np.isnan(image)]

				# Get moment/centroid of object
				M = cv2.moments(self.mask3)

				# Calculate center of object and find error from center object
				# Centroid not found
				if (M['m00'] == 0):
					rospy.loginfo('Not tracking ')
					self.move_cmd.linear.x = 0
					self.move_cmd.angular.z = 0

				#Centroid found:
				if ( M['m00'] > 0):
					cx = int(M['m10']/M['m00'])
					cy = int(M['m01']/M['m00'])
					centerOfObject = (int(cx),int(cy))
					dx = cx - col/2 # +ve move left, -ve move right?
					dy = cy - rows/2
					depth = np.median(image)
					rospy.loginfo('depth is '+ str(depth))

					#dz range can go from 1100
					dz = depth - self.desired_thresh

					#Ignore very low depth values
					if depth <= self.invalid_thresh:
						self.move_cmd.linear.x = 0
						self.move_cmd.angular.z = K*(0)*dx

					#If below low bound but higher than invalid then move backwards?
					elif depth < self.desired_lowBound:
						if (abs(Kx*dz) < self.max_speed):
							self.move_cmd.linear.x = Kx*dz
						else:
							self.move_cmd.linear.x = -self.max_speed

						self.move_cmd.angular.z = K*(-1)*dx

					#If below upper bound but higher than low bound dont move forward but do rotate
					elif depth < self.desired_upBound:
						self.move_cmd.linear.x = 0*Kx
						self.move_cmd.angular.z = K*(-1)*dx

					#If below the invalid thresh then move forward
					elif depth < self.invalid_max:
						if (Kx*dz < self.max_speed):
							self.move_cmd.linear.x = Kx*dz
						else:
							self.move_cmd.linear.x = -self.max_speed

						self.move_cmd.angular.z = K*(-1)*dx

					#If above threshold do nothing
					elif depth >= self.invalid_max:
						self.move_cmd.linear.x = 0
						self.move_cmd.angular.z = K*(0)*dx
					else:
						self.move_cmd.linear.x = 0
						self.move_cmd.angular.z = K*(0)*dx
					

				self.cmd_vel.publish(self.move_cmd)

				self.r.sleep()
			else:
				# do nothing
				rospy.loginfo('do nothing')

				self.move_cmd.linear.x = 0.0
				self.move_cmd.angular.z = 0
				self.cmd_vel.publish(self.move_cmd)
				self.r.sleep()

		except CvBridgeError, e:
			print e

  
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
		self.magnitude = sqrt(self.x**2 + self.y**2)

	def callbackImage(self,data):
		try:
		  if self.emergency == 1:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
			#cv2.imshow("color_camera_msg.jpg", cv_image)
			if self.savePic == 1:
			  cv2.imwrite("UserSnapshot.jpg",cv_image)
			  # send to server
			  url = 'http://128.61.14.57:3000/image'  #<-------------------SERVER IP ADDRESS HERE------------
			  files ={'image':open('UserSnapshot.jpg','rb')}
			  sender = requests.post(url, files=files)
			  self.savePic = 0
			#cv2.waitKey(5000)
			#rospy.loginfo(imgName)

		except CvBridgeError, e:
		  print e

	def shutdown(self):
		# stop turtlebot
		rospy.loginfo("Stop TurtleBot")
		# a default Twist has linear.x of 0 and angular.z of 0.  So it'll stop TurtleBot
		self.cmd_vel.publish(Twist())
		# sleep just makes sure TurtleBot receives the stop command prior to shutting down the script
		rospy.sleep(1)
	
def main(args):
	ic = following_final2()
	#rospy.init_node('image_converter', anonymous=True)
	try:

		rospy.spin()
	except KeyboardInterrupt:
		print "Shutting down"

if __name__ == '__main__':
		main(sys.argv)