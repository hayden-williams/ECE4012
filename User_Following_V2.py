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
	invalid_thresh = 300
	desired_thresh = 1500
	invalid_max = 2000


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
			K = 0.009
			# Kx is for movment in x direction (LEFT AND RIGHT)
			Kx = 1


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
			#rospy.loginfo(cR)
			#rospy.loginfo(cC)

			# How Large our field of view for our desired object i.e where we looking in the picture for our object
			# Mask to remove any uneccesary information outside our looking area.
			rowFrac = np.int(np.round(.25*cR))
			colFrac = np.int(np.round(.4*cC))
			#rospy.loginfo(rowFrac)
			#rospy.loginfo(colFrac)
			self.mask2 =  np.zeros((rows,col))
			self.mask2[cR+rowFrac:cR+(2*rowFrac),cC-colFrac:cC+colFrac] = 5
			#self.mask2[1,:] = 5
			self.mask2 = np.uint16(self.mask2)
			self.mask2 = cv2.inRange(self.mask2,np.array(4,dtype = "uint16"),np.array(6,dtype = "uint16"))

			# Mask to get values of specific box in z direction only interested in our object/person
			min_z= np.array(500, dtype = "uint16") #bgr
			max_z= np.array(2000, dtype = "uint16")
			self.mask = cv2.inRange(self.depth_image, min_z, max_z)
			self.mask3 = cv2.bitwise_and(self.mask,self.mask, mask= self.mask2)
			image = cv2.bitwise_and(self.depth_image,self.depth_image, mask= self.mask3)
			image = image.astype(float)
			image[image==0] = np.nan
			#rospy.loginfo(self.depth_image[cR,cC])


				#cv2.imshow('image',image)
				#cv2.waitKey(5)

				
				# Convert BGR to GReyscale





			# Get moment/centroid of object
			M = cv2.moments(self.mask3)
			#rospy.loginfo('depth image center = ' + str(self.depth_image[cR,cC]))
			#rospy.loginfo('depth image center = ' + str(np.uint8(self.depth_image[cR,cC])))
			#rospy.loginfo('mask sum is ' + str(np.sum(self.mask)))
			#rospy.loginfo(np.sum(self.mask2))
			#rospy.loginfo(np.sum(self.mask3))
			#height, width, channels = image.shape #grey scale channel is 1, rgb is 3

			# Calculate center of object and find error from center object

			if ( M['m00'] > 0):
				cx = int(M['m10']/M['m00'])
				cy = int(M['m01']/M['m00'])
				centerOfObject = (int(cx),int(cy))
				#rospy.loginfo('center of object is ' + str(centerOfObject))
				#rospy.loginfo(' cx is ' + str(cx))

				#cv2.circle(image,centerOfObject,10,(0,255,0),-1)
				#rospy.loginfo("in if statement in callback")
				dx = cx - col/2 # +ve move left, -ve move right?
				dy = cy - rows/2
				#rospy.loginfo('dx is '+ str(dx))
				depth = np.median(image)
				rospy.loginfo('mean is ' + str(np.mean(image)))
				#rospy.loginfo(image[cR,cC])
				rospy.loginfo('median is '+ str(depth))

				#Movement code to center object and keep desired distance
					#self.move_cmd.linear.x = 0.0015*(-1)*dy


					#distance = self.depth_image(cx,cy)
					#print self.depth_image[cx,cy]
					#print "hello"
				#rospy.loginfo('object depth '+ str(self.depth_image[cx,cy]))
				if self.depth_image[cx,cy] <= self.invalid_thresh:
					self.move_cmd.linear.x = 0
					self.move_cmd.angular.z = K*(0)*dx
				elif self.depth_image[cx,cy] < self.desired_thresh:
					self.move_cmd.linear.x = -0.0*Kx
					self.move_cmd.angular.z = K*(-1)*dx
				elif self.depth_image[cx,cy] < self.desired_thresh:
					self.move_cmd.linear.x = 0
					self.move_cmd.angular.z = K*(-1)*dx
				elif self.depth_image[cx,cy] >= self.invalid_max:
					self.move_cmd.linear.x = 0.0*Kx
					self.move_cmd.angular.z = K*(0)*dx
				else:
					self.move_cmd.linear.x = 0
					self.move_cmd.angular.z = K*(0)*dx
				
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