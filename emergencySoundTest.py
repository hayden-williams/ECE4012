# Stephen Williams
# Created 16 April 2018
# Edited 16 April 2018

# This code created to test sound functionality on turtlebot

import rospy
import roslib; roslib.load_manifest('sound_play')
import sys
from sound_play.libsoundplay import SoundClient



if __name__ == '__main__':
	counter = 0
	soundCounter = 0

	rospy.init_node('emergency', anonymous=False)
	rate = rospy.Rate(10)

	soundhandle = SoundClient()
	rospy.sleep(2)

	while(1):
		counter = counter + 1
		if soundCounter%1000 == 0:
			# Adjust the number so this says Emergency repeatedly
			soundCounter = 0
			soundhandle.say('Emergency')
			soundhandle.stopSaying(self, 'Emergency')
		soundCounter = soundCounter + 1

	print('While loop ended')