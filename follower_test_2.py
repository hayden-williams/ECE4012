#!/usr/bin/env python

"""
    follower.py - Version 1.0 2012-06-01
    
    Follow a "person" by tracking the nearest object in x-y-z space.
    
    Based on the follower application by Tony Pratkanis at:
    
    http://ros.org/wiki/turtlebot_follower
    
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2012 Patrick Goebel.  All rights reserved.
    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
"""

# Modified by Edgardo Marchand for Georgia Tech ECE Senior Design Spring 2018


import roslib
import rospy
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Twist
from math import copysign
import point_cloud2

class Follower():
    def __init__(self):
        rospy.init_node("follower")
        
        # Set the shutdown function (stop the robot)
        rospy.on_shutdown(self.shutdown)
        
        # The dimensions (in meters) of the box in which we will search
        # for the person (blob). These are given in camera coordinates
        # where x is left/right,y is up/down and z is depth (forward/backward)
        self.min_x = rospy.get_param("~min_x", -0.35)
        self.max_x = rospy.get_param("~max_x", 0.35)
        self.min_y = rospy.get_param("~min_y", 0.2)
        self.max_y = rospy.get_param("~max_y", 0.6)
        self.max_z = rospy.get_param("~max_z", 2)
        
        
        # The goal distance (in meters) to keep between the robot and the person
        self.goal_z = rospy.get_param("~goal_z", .6)
        
        # How far away from the goal distance (in meters) before the robot reacts
        self.z_threshold = rospy.get_param("~z_threshold", 0.1)
        
        # How far away from being centered (x displacement) on the person
        # before the robot reacts
        self.x_threshold = rospy.get_param("~x_threshold", 0.05)
        
        # How much do we weight the goal distance (z) when making a movement
        self.z_scale = rospy.get_param("~z_scale", .5)

        # How much do we weight x-displacement of the person when making a movement        
        self.x_scale = rospy.get_param("~x_scale", 1.2)
        
        # The maximum rotation speed in radians per second
        self.max_angular_speed = rospy.get_param("~max_angular_speed", 1.5)
        
        # The minimum rotation speed in radians per second
        self.min_angular_speed = rospy.get_param("~min_angular_speed", 0.0)
        
        # The max linear speed in meters per second
        self.max_linear_speed = rospy.get_param("~max_linear_speed", 0.5)
        
        # The minimum linear speed in meters per second
        self.min_linear_speed = rospy.get_param("~min_linear_speed", 0.0)
    
        # Publisher to control the robot's movement
        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
        print 'before sub'

        rospy.Subscriber('/camera/depth_registered/points', PointCloud2, self.set_cmd_vel)
        
        # Wait for the pointcloud topic to become available
        print 'before wait'
        rospy.wait_for_message('/camera/depth_registered/points', PointCloud2)
        
    def set_cmd_vel(self, msg):
        # Initialize the centroid coordinates point count
        x = y = z = n = 0
        
        
        # Read in the x, y, z coordinates of all points in the cloud
        for point in point_cloud2.read_points(msg, skip_nans=True):
            pt_x = point[0]
            pt_y = point[1]
            pt_z = point[2]
            
            
            # Keep only those points within our designated boundaries and sum them up
            if -pt_y > self.min_y and -pt_y < self.max_y and  pt_x < self.max_x and pt_x > self.min_x and pt_z < self.max_z:
                x += pt_x
                y += pt_y
                z += pt_z
                n += 1
                
        # Stop the robot by default
        move_cmd = Twist()
        
        # If we have points, compute the centroid coordinates
        rospy.loginfo("n is "+str(n))
        if n:    
            x /= n 
            y /= n 
            z /= n

            
                        
            # Check our movement thresholds
            if (abs(z - self.goal_z) > self.z_threshold) or (abs(x) > self.x_threshold):     
                # Compute the linear and angular components of the movement
                linear_speed = (z - self.goal_z) * self.z_scale
                angular_speed = -x * self.x_scale
               
                
                # Make sure we meet our min/max specifications
                linear_speed = copysign(max(self.min_linear_speed, 
                                            min(self.max_linear_speed, abs(linear_speed))), linear_speed)
                angular_speed = copysign(max(self.min_angular_speed, 
                                             min(self.max_angular_speed, abs(angular_speed))), angular_speed)
    
                move_cmd.linear.x = linear_speed
                move_cmd.angular.z = angular_speed
                        
        # Publish the movement command

        self.cmd_vel_pub.publish(move_cmd)

        
    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)     
                   
if __name__ == '__main__':
    try:
        Follower()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Follower node terminated.")