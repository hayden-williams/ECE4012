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

class obstacle_detect():
	#Threshold for detecting object in a zone
	#Zones go from left to right on image 
	# Distance in mm
	z_thresh = 800
	z_threshCorner = z_thresh
	ZoneList = np.array([0,0,0,0,0,0])




	def __init__(self):
		rospy.init_node('image_converter', anonymous=True)
		# Initialize bridge
		self.bridge = CvBridge()
		# Subscribe to depth sensor and get raw image
		self.depth_sub = rospy.Subscriber("/camera/depth/image_raw",Image,self.callback)
		#self.depth_sub = rospy.Subscriber("/camera/depth_registered/image_raw",Image,self.callback_depth)
		# Publish to navigation to move robot

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
			rospy.loginfo("Zone List is "+ str(self.ZoneList))




				


			self.r.sleep()


		except CvBridgeError, e:
			print e




	
def main(args):
	od = obstacle_detect()
	#rospy.init_node('image_converter', anonymous=True)
	try:

		rospy.spin()
	except KeyboardInterrupt:
		print "Shutting down"

if __name__ == '__main__':
		main(sys.argv)