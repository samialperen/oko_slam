#! /usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

pub = None #Since we are using publisher in the functions as well we need to 
# make it global

def callback_laser(msg):
    # Our laser publishes 64 point per rotation
    # Let's divide these 64 point to total_region_number different region
    # LIDAR rotates counter-clockwise
    # Let's start from the absolute right direction region and rotate cclockwise    
    # By taking mean value in reach region, we want to find out which region
    # has the closest object/wall 
    # To eliminate Inf measurement, we used min function  
    MAX_LIDAR_RANGE = 3 #in meter
    total_region_number = 4 #East, North, West, South
    # Which regions corresponds to which side will be found later.
    regions = { 
      'east': min(min(msg.ranges[0:15]), MAX_LIDAR_RANGE), 
      'north': min(min(msg.ranges[16:31]), MAX_LIDAR_RANGE), 
      'west': min(min(msg.ranges[32:47]), MAX_LIDAR_RANGE), 
      'south': min(min(msg.ranges[48:63]), MAX_LIDAR_RANGE)
     }
    #rospy.loginfo(regions) #Debug purpose
    
    take_action(regions)
 
def take_action(regions):
    msg = Twist()
    linear_x = 0
    angular_z = 0
    max_dist2robot = 0.30 #If detected object is far away from this value, we 
    # don't consider it
    linear_velocity = 0.2
    angular_velocity = 0.15
    
    state_description = ''
    
    # If there exists an object only north then turn left --> Case 2
    # Positive turn around z axis corresponds to turning left
    if regions['north'] > max_dist2robot and regions['west'] > max_dist2robot and regions['east'] > max_dist2robot:
        state_description = 'case 1 - nothing'
        linear_x = linear_velocity
        angular_z = 0
    elif regions['north'] < max_dist2robot and regions['west'] > max_dist2robot and regions['east'] > max_dist2robot:
        state_description = 'case 2 - north'
        linear_x = 0
        angular_z = angular_velocity
    elif regions['north'] > max_dist2robot and regions['west'] > max_dist2robot and regions['east'] < max_dist2robot:
        state_description = 'case 3 - east'
        linear_x = 0
        angular_z = angular_velocity
    elif regions['north'] > max_dist2robot and regions['west'] < max_dist2robot and regions['east'] > max_dist2robot:
        state_description = 'case 4 - west'
        linear_x = 0
        angular_z = -angular_velocity
    elif regions['north'] < max_dist2robot and regions['west'] > max_dist2robot and regions['east'] < max_dist2robot:
        state_description = 'case 5 - north and east'
        linear_x = 0
        angular_z = angular_velocity
    elif regions['north'] < max_dist2robot and regions['west'] < max_dist2robot and regions['east'] > max_dist2robot:
        state_description = 'case 6 - north and west'
        linear_x = 0
        angular_z = -angular_velocity
    elif regions['north'] < max_dist2robot and regions['west'] < max_dist2robot and regions['east'] < max_dist2robot:
        state_description = 'case 7 - north and west and east'
        linear_x = 0
        angular_z = angular_velocity
    elif regions['north'] > max_dist2robot and regions['west'] < max_dist2robot and regions['east'] < max_dist2robot:
        state_description = 'case 8 - west and east'
        linear_x = linear_velocity
        angular_z = 0
    else:
        state_description = 'Undefined Case'
        rospy.loginfo(regions)

    rospy.loginfo(state_description)
    #rospy.loginfo(regions)
    msg.linear.x = -linear_x    
    msg.angular.z = -angular_z
    pub.publish(msg)    

 
 
def main():
    global pub
    
    rospy.init_node('obstacle_avoidance') #Add namespace to launch file later
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    
    sub= rospy.Subscriber("/scan", LaserScan, callback_laser)

    rospy.spin()

if __name__ == '__main__':
    main()
    
    


