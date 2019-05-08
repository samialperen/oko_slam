#! /usr/bin/env python

# import ros stuff
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations
from std_srvs.srv import *

import math
import numpy as np

## Global variables
active_ = False #True for our previous case
pub_ = None #Since we are using publisher in the functions as well we need to 
# make it global

linear_velocity_ = 0.03
angular_velocity_ = 0.20

regions_ = {
        'east': 0,
        'north': 0,
        'west': 0,
        'south': 0,
        'n-w': 0, #North West
        'n-e': 0, #North East
}

state_ = 0
state_dict_ = {
    0: 'find the wall',
    1: 'turn left',
    2: 'follow the wall',
    3: 'turn right',
}

def wall_follower_switch(req):
    global active_
    active_ = req.data
    res = SetBoolResponse()
    res.success = True
    res.message = 'Done!'
    return res

def callback_laser(msg):
    global regions_

    # Our laser publishes 128 point per rotation
    # Let's divide these 128 point to total_region_number different region
    # LIDAR rotates counter-clockwise
    # Let's start from the absolute right direction region and rotate cclockwise    
    # By taking mean value in reach region, we want to find out which region
    # has the closest object/wall 
    # To eliminate Inf measurement, we used mean function  

    MAX_LIDAR_RANGE = 3 #in meter
    total_region_number = 6 #East, North, West, South
#### Simulation
    regions_ = { 
      'east': min(min(msg.ranges[0:31]), MAX_LIDAR_RANGE), 
      'north': min(min(msg.ranges[32:63]), MAX_LIDAR_RANGE), 
      'west': min(min(msg.ranges[64:95]), MAX_LIDAR_RANGE), 
      'south': min(min(msg.ranges[96:127]), MAX_LIDAR_RANGE)
      #'n-e': min(min(msg.ranges[96:127]), MAX_LIDAR_RANGE),
     }
     
     

###### Real World
#    east_array = msg.ranges[124:127] + msg.ranges[0:3]
#    north_array = msg.ranges[124:127] + msg.ranges[0:3]
#
#    north_east_west = np.mean(msg.ranges[115:118])
#    north_east = np.mean(msg.ranges[110:114])
#    north_east_east = np.mean(msg.ranges[106:109])

#    north_west_west = np.mean(msg.ranges[19:22])
#    north_west = np.mean(msg.ranges[14:15])
#    north_west_east = np.mean(msg.ranges[10:13])

## Regions when we use cable electricity
#    regions_ = { 
#      'north': np.mean(msg.ranges[28:36]),
#      'west': np.mean(msg.ranges[60:68]),
#      'south': np.mean(msg.ranges[92:100]),
#      'east': np.mean(east_array),
#      'n-e': np.mean(msg.ranges[10:22]),
#      'n-w': np.mean(msg.ranges[42:54])
#     }
## Regions when we use lipo battery 
#    regions_ = {
#      'south': np.mean(msg.ranges[60:68]),
#      'east':  np.mean(msg.ranges[97:102]),
#      'west': np.mean(msg.ranges[23:38]),
#      'north': np.mean(north_array),
#      'n-e': min(north_east_east, north_east, north_east_west),
#      'n-w': min(north_west_east, north_west, north_west_west),
#     }

    take_action()
    
def change_state(state):
    global state_, state_dict_
    if state is not state_:
        print 'Wall follower - [%s] - %s' % (state, state_dict_[state])
        state_ = state

def take_action():
    global regions_, linear_velocity_, angular_velocity_
    regions = regions_
    msg = Twist()
    linear_x = 0
    angular_z = 0
    state_description = ''

    max_dist2robot = 0.30
    min_dist2robot = 0.092
    max_dist2obj = 0.25
    min_dist2obj = 0.092

    # If there exists an object only north then turn left --> Case 2
    # Positive turn around z axis corresponds to turning left

#    if regions['n-w'] < max_dist2obj and regions['n-e'] > max_dist2obj and regions['n-w'] > min_dist2obj:
#        state_description = 'case 9 nw'
#	change_state(3)
#    elif regions['n-w'] > max_dist2obj and regions['n-e'] < max_dist2obj and regions['n-e'] > min_dist2obj :
#        state_description = 'case 10 ne'
#	change_state(1)
#    elif regions['n-w'] < max_dist2obj and regions['n-e'] < max_dist2obj and regions['n-w'] > min_dist2obj and regions['n-e'] > min_dist2obj:
#        state_description = 'case 11 ne nw'
#        change_state(1)
#    else:
#        if regions['north'] > max_dist2robot and regions['west'] > max_dist2robot and regions['east'] > max_dist2robot:
#            state_description = 'case 1 - nothing'
#            change_state(0)
#        elif regions['north'] < max_dist2robot and regions['west'] > max_dist2robot and regions['east'] > max_dist2robot and regions['north'] > min_dist2robot:
#            state_description = 'case 2 - north'
#            change_state(3)
#        elif regions['north'] > max_dist2robot and regions['west'] > max_dist2robot and regions['east'] < max_dist2robot and regions['east'] > min_dist2robot:
#            state_description = 'case 3 - east'
#            change_state(2)
#        elif regions['north'] > max_dist2robot and regions['west'] < max_dist2robot and regions['east'] > max_dist2robot and regions['west'] > min_dist2robot:
#            state_description = 'case 4 - west'
#            change_state(2)
#        elif regions['north'] < max_dist2robot and regions['west'] > max_dist2robot and regions['east'] < max_dist2robot and regions['east'] > min_dist2robot and regions['north'] > min_dist2robot:
#            state_description = 'case 5 - north and east'
#            change_state(1)
#        elif regions['north'] < max_dist2robot and regions['west'] < max_dist2robot and regions['east'] > max_dist2robot and regions['west'] > min_dist2robot and regions['north'] > min_dist2robot:
#            state_description = 'case 6 - north and west'
#            change_state(3)
#        elif regions['north'] < max_dist2robot and regions['west'] < max_dist2robot and regions['east'] < max_dist2robot and regions['east'] > min_dist2robot and regions['west'] > min_dist2robot and regions['north'] > min_dist2robot:
#            state_description = 'case 7 - north and west and east'
#            change_state(3)
#        elif regions['north'] > max_dist2robot and regions['west'] < max_dist2robot and regions['east'] < max_dist2robot and regions['east'] > min_dist2robot and regions['west'] > min_dist2robot:
#            state_description = 'case 8 - west and east'
#            change_state(0)
#        else:
#            state_description = 'Unknown Case GG WP!'
#            rospy.loginfo(regions)
        
    #rospy.loginfo(state_description)
    #rospy.loginfo(regions)

def find_wall():
    msg = Twist()
    msg.linear.x = linear_velocity_
    msg.angular.z = 0
    return msg

def turn_left():
    msg = Twist()
    msg.angular.z = angular_velocity_
    msg.linear.x = 0
    return msg

def turn_right():
    msg = Twist()
    msg.angular.z = -angular_velocity_
    msg.linear.x = 0
    return msg

def follow_the_wall():
    global regions_
    
    msg = Twist()
    msg.linear.x = linear_velocity_
    msg.angular.z = 0
    return msg
    
# Stop sending commands when you shut down the node    
def sd_hook(): 
    msg = Twist()
    msg.linear.x = 0
    msg.angular.z = 0
    pub_.publish(msg)    

def main():
    global pub_, active_
    
    rospy.init_node('follow_wall')
    
    pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    
    sub = rospy.Subscriber('/scan', LaserScan, callback_laser)
    
    srv = rospy.Service('wall_follower_switch', SetBool, wall_follower_switch)

    rospy.on_shutdown(sd_hook)
    
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        if not active_:
            rate.sleep()
            continue
        
        msg = Twist()
        if state_ == 0:
            msg = find_wall()
            #print "Find wall Bitch!"
        elif state_ == 1:
            msg = turn_left()
            #print "Turn Left Bitch!"
	elif state_ == 3:
            msg = turn_right()
            #print "Turn Right Bitch!"
        elif state_ == 2:
            msg = follow_the_wall()
            #print "Follow the Wall Bitch!"
            pass
        else:
            rospy.logerr('Unknown state! GG WP :(')
        
        #pub_.publish(msg)
        
        rate.sleep()

if __name__ == '__main__':
    main()
