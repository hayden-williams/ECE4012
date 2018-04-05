# Created by Edgardo Marchand
# Created 4/4/2018
# Senior Design Spring 2018 Georgia Institute of Technology

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

class image_converter:
	#Threshold for detecting object in a zone
	#Zones go from left to right on image 
	# Distance in mm




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
			


			# Get Image and find size of image
			self.depth_image = self.bridge.imgmsg_to_cv2(data, "passthrough")
			rows, col, channels = self.depth_image.shape #grey scale channel is 1, rgb is 3

			# Find Center of Image
			cR = np.int(np.round(rows/2))
			cC = np.int(np.round(col/2))


			colFrac = np.int(np.round(.25*col))


			self.mask =  np.zeros((rows,col))
			self.mask[cR-rows*.25:rows,cC-colFrac:cC+colFrac] = 5
			self.mask = np.uint16(self.mask)
			self.mask = cv2.inRange(self.mask,np.array(4,dtype = "uint16"),np.array(6,dtype = "uint16"))


			min_z= np.array(100, dtype = "uint16") #bgr
			max_z= np.array(700, dtype = "uint16")
			self.mask2 = cv2.inRange(self.depth_image, min_z, max_z)
			
			#Combination of masks
			self.mask3 = cv2.bitwise_and(self.mask,self.mask, mask= self.mask2)

			self.maskZone1 = np.zeros((rows,col))
			self.maskZone1[0:rows,0:col*.25] = 5
			self.maskZone1 = np.uint16(self.maskZone1)
			self.maskZone1 = cv2.inRange(self.maskZone1,np.array(4,dtype = "uint16"),np.array(6,dtype = "uint16"))
			self.Zone1 = cv2.bitwise_and(self.mask3,self.mask3, mask= self.maskZone1)

			self.maskZone2 = np.zeros((rows,col))
			self.maskZone2[0:rows,col*.25+1:cC] = 5
			self.maskZone2 = np.uint16(self.maskZone2)
			self.maskZone2 = cv2.inRange(self.maskZone2,np.array(4,dtype = "uint16"),np.array(6,dtype = "uint16"))
			self.Zone2 = cv2.bitwise_and(self.mask3,self.mask3, mask= self.maskZone2)

			self.maskZone3 = np.zeros((rows,col))
			self.maskZone3[0:rows,cC+1:cC+col*.25] = 5
			self.maskZone3 = np.uint16(self.maskZone3)
			self.maskZone3 = cv2.inRange(self.maskZone3,np.array(4,dtype = "uint16"),np.array(6,dtype = "uint16"))
			self.Zone3 = cv2.bitwise_and(self.mask3,self.mask3, mask= self.maskZone3)

			self.maskZone4 = np.zeros((rows,col))
			self.maskZone4[0:rows,cC+col*.25+1:col] = 5
			self.maskZone4 = np.uint16(self.maskZone4)
			self.maskZone4 = cv2.inRange(self.maskZone4,np.array(4,dtype = "uint16"),np.array(6,dtype = "uint16"))
			self.Zone4 = cv2.bitwise_and(self.mask3,self.mask3, mask= self.maskZone4)

			sumZone1 = np.sum(self.Zone1 / 255)
			rospy.loginfo("sum of Zone1 is " + str(sumZone1))
			sumZone2 = np.sum(self.Zone2 / 255)
			rospy.loginfo("sum of Zone2 is " + str(sumZone2))
			sumZone3 = np.sum(self.Zone3 / 255)
			rospy.loginfo("sum of Zone3 is " + str(sumZone3))
			sumZone4 = np.sum(self.Zone4 / 255)
			rospy.loginfo("sum of Zone4 is " + str(sumZone4))

			self.move_cmd.linear.x = 0
			self.move_cmd.angular.z = 0






				

			self.cmd_vel.publish(self.move_cmd)

			self.r.sleep()


		except CvBridgeError, e:
			print e




	
def main(args):
	ic = image_converter()
	#rospy.init_node('image_converter', anonymous=True)
	try:

		rospy.spin()
	except KeyboardInterrupt:
		print "Shutting down"

if __name__ == '__main__':
		main(sys.argv)