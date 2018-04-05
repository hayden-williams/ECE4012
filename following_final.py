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

import roslib
import sys 
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np 
from geometry_msgs.msg import Twist

class following_final():
	# Distance in mm
	invalid_thresh = 300
	desired_thresh = 1000
	desired_lowBound = 950
	desired_upBound = 1050
	invalid_max = 2000
	max_speed = 1


	def __init__(self):
		rospy.init_node('image_converter', anonymous=True)
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

	def callback(self,data):
		try:
			
			# Gain Values for movement

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
			self.mask2[cR+rowFrac:cR+(2*rowFrac),cC-colFrac:cC+colFrac] = 5
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


		except CvBridgeError, e:
			print e




	
def main(args):
	ic = following_final()
	#rospy.init_node('image_converter', anonymous=True)
	try:

		rospy.spin()
	except KeyboardInterrupt:
		print "Shutting down"

if __name__ == '__main__':
		main(sys.argv)