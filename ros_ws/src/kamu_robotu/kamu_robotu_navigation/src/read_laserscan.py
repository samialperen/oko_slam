#! /usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan

def callback_laser(msg):
    # Our laser publishes 64 point per rotation
    # Let's divide these 64 point to 8 different region
    # LIDAR rotates counter-clockwise
    # Let's start from the absolute right direction region and rotate cclockwise    
    # By taking mean value in reach region, we want to find out which region
    # has the closest object/wall 
    # To eliminate Inf measurement, we used min function  
    MAX_LIDAR_RANGE = 3 #in meter
    regions = [ 
      min(np.mean(msg.ranges[0:7]), MAX_LIDAR_RANGE), #Region1 - (East) (right)
      min(np.mean(msg.ranges[8:15]), MAX_LIDAR_RANGE), #Region2 - (North East)
      min(np.mean(msg.ranges[16:23]), MAX_LIDAR_RANGE), # North
      min(np.mean(msg.ranges[24:31]), MAX_LIDAR_RANGE), # North West
      min(np.mean(msg.ranges[32:39]), MAX_LIDAR_RANGE), # West
      min(np.mean(msg.ranges[40:47]), MAX_LIDAR_RANGE), # South West
      min(np.mean(msg.ranges[48:55]), MAX_LIDAR_RANGE), # South 
      min(np.mean(msg.ranges[56:63]), MAX_LIDAR_RANGE), # South East
     ]
    rospy.loginfo(regions)

def main():
    rospy.init_node('read_laserscan') #Add namespace to launch file later
    sub= rospy.Subscriber("/scan", LaserScan, callback_laser)

    rospy.spin()

if __name__ == '__main__':
    main()
