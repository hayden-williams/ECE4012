# Stephen Williams and Edgardo Marchand
# Created 3 Dec 2017

# Homework 13
#!/usr/bin/env python
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
	# Distance in mm
	max_stop = 1000 # above this go forward
	min_stop = 500 # below this go backwards
	no_below = 400 # stop if below this

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
			#print "Hello"
			# Gain Values for movement
			# Speed Gain
			K = 0.001
			# Kx is for movment in x direction (LEFT AND RIGHT)
			Kx = 1

			# How far we looking for object in depth image (2 meters)
			z_threshold = np.uint8(2)

			# Get Image and find size of image
			self.depth_image = self.bridge.imgmsg_to_cv2(data, "passthrough")
			#cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
			#hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
			rows, col, channels = self.depth_image.shape #grey scale channel is 1, rgb is 3
			#rows = self.bridge.imgmsg_to_cv2(height, "passthrough")
			#col = self.bridge.imgmsg_to_cv2(width, "passthrough")

			# Find Center of Image
			cR = np.int(np.round(rows/2))
			cC = np.int(np.round(col/2))

			# How Large our field of view for our desired object i.e where we looking in the picture for our object
			# Mask to remove any uneccesary information outside our looking area.
			rowFrac = np.int(np.round(.20*cR))
			colFrac = np.int(np.round(.20*cC))
			self.mask2 =  np.zeros((rows,col))
			self.mask2[cR-rowFrac:cR+rowFrac,cC-colFrac:cC+colFrac] = 5
			#self.mask2[1,:] = 5
			self.mask2 = np.uint8(self.mask2)
			self.mask2 = cv2.inRange(self.mask2,np.array(4,dtype = "uint8"),np.array(6,dtype = "uint8"))

			# Mask to get values of specific box in z direction only interested in our object/person
			min_z= np.array(100, dtype = "uint8") #bgr
			max_z= np.array(500, dtype = "uint8")
			self.mask = cv2.inRange(np.uint8(self.depth_image), np.array(100, dtype = "uint8"), np.array(3000,dtype="uint8"))
			image = cv2.bitwise_and(self.depth_image,self.depth_image, mask= self.mask)
			image = cv2.bitwise_and(image,image, mask= self.mask2)
			#rospy.loginfo(self.mask[cR,cC])


				#cv2.imshow('image',image)
				#cv2.waitKey(5)

				
				# Convert BGR to GReyscale





			# Get moment/centroid of object
			M = cv2.moments(self.mask)
			#height, width, channels = image.shape #grey scale channel is 1, rgb is 3

			# Calculate center of object and find error from center object

			if ( M['m00'] > 0):
				cx = int(M['m10']/M['m00'])
				cy = int(M['m01']/M['m00'])
				centerOfObject = (int(cx),int(cy))
				#cv2.circle(image,centerOfObject,10,(0,255,0),-1)
				#rospy.loginfo("in if statement in callback")
				dx = cx - col/2 # +ve move left, -ve move right?
				dy = cy - rows/2
				rospy.loginfo(dx)

				#Movement code to center object and keep desired distance
					#self.move_cmd.linear.x = 0.0015*(-1)*dy
				self.move_cmd.angular.z = K*(1)*dx

					#distance = self.depth_image(cx,cy)
					#print self.depth_image[cx,cy]
					#print "hello"
				rospy.loginfo(self.depth_image[cx,cy])
				if self.depth_image[cx,cy] <= self.no_below:
					self.move_cmd.linear.x = 0
				elif self.depth_image[cx,cy] < self.min_stop:
					self.move_cmd.linear.x = -0.0*Kx
				elif self.depth_image[cx,cy] < self.max_stop:
					self.move_cmd.linear.x = 0
				elif self.depth_image[cx,cy] >= self.max_stop:
					self.move_cmd.linear.x = 0.0*Kx
				else:
					self.move_cmd.linear.x = 0
				
				#rospy.loginfo("in callback")
			self.cmd_vel.publish(self.move_cmd)

			self.r.sleep()
				

				#cv2.imshow('mask',mask)

				#greyImage = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
				#ret,thresh = cv2.threshold(grey,127,255,0)


				

				#cv2.imshow("color_camera_msg.jpg", cv_image)
				#cv2.waitKey(3)
				#print "image saved!"

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