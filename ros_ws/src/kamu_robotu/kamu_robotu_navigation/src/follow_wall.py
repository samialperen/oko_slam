#! /usr/bin/env python
# We are looking for the wall. If we find wall, we turn left to keep wall always
# right side of the robot

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations
from std_srvs.srv import *

import math

## Global variables
active_ = True
pub_ = None #Since we are using publisher in the functions as well we need to 
# make it global

linear_velocity_ = 0.2
angular_velocity_ = 0.15

regions_ = {
        'east': 0,
        'north': 0,
        'west': 0,
        'south': 0,
}

state_ = 0 #Start from find the wall state
state_dict_ = {
    0: 'find the wall',
    1: 'turn left',
    2: 'follow the wall',
}

def wall_follower_switch(req):
    global active_
    active_ = req.data
    res = SetBoolResponse()
    res.success = True
    res.message = 'Done!'
    return res


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
    regions_ = { 
      'east': min(min(msg.ranges[0:15]), MAX_LIDAR_RANGE), 
      'north': min(min(msg.ranges[16:31]), MAX_LIDAR_RANGE), 
      'west': min(min(msg.ranges[32:47]), MAX_LIDAR_RANGE), 
      'south': min(min(msg.ranges[48:63]), MAX_LIDAR_RANGE)
     }
    
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
    
    max_dist2robot = 0.50 #If detected object is far away from this value, we 
    # don't consider it
    
    # If there exists an object only north then turn left --> Case 2
    # Positive turn around z axis corresponds to turning left
    if regions['north'] > max_dist2robot and regions['west'] > max_dist2robot and regions['east'] > max_dist2robot:
        state_description = 'case 1 - nothing'
        linear_x = linear_velocity_
        angular_z = 0
    elif regions['north'] < max_dist2robot and regions['west'] > max_dist2robot and regions['east'] > max_dist2robot:
        state_description = 'case 2 - north'
        linear_x = 0
        angular_z = angular_velocity_
    elif regions['north'] > max_dist2robot and regions['west'] > max_dist2robot and regions['east'] < max_dist2robot:
        state_description = 'case 3 - east'
        linear_x = 0
        angular_z = angular_velocity_
    elif regions['north'] > max_dist2robot and regions['west'] < max_dist2robot and regions['east'] > max_dist2robot:
        state_description = 'case 4 - west'
        linear_x = 0
        angular_z = -angular_velocity_
    elif regions['north'] < max_dist2robot and regions['west'] > max_dist2robot and regions['east'] < max_dist2robot:
        state_description = 'case 5 - north and east'
        linear_x = 0
        angular_z = angular_velocity_
    elif regions['north'] < max_dist2robot and regions['west'] < max_dist2robot and regions['east'] > max_dist2robot:
        state_description = 'case 6 - north and west'
        linear_x = 0
        angular_z = -angular_velocity_
    elif regions['north'] < max_dist2robot and regions['west'] < max_dist2robot and regions['east'] < max_dist2robot:
        state_description = 'case 7 - north and west and east'
        linear_x = 0
        angular_z = angular_velocity_
    elif regions['north'] > max_dist2robot and regions['west'] < max_dist2robot and regions['east'] < max_dist2robot:
        state_description = 'case 8 - west and east'
        linear_x = linear_velocity_
        angular_z = 0
    else:
        state_description = 'Unknown Case GG WP!'
        rospy.loginfo(regions)
      

def find_wall():
    msg = Twist()
    msg.linear.x = -linear_velocity_
    msg.angular.z = -angular_velocity_
    return msg

def turn_left():
    msg = Twist()
    msg.angular.z = angular_velocity_
    return msg

def follow_the_wall():
    global regions_
    
    msg = Twist()
    msg.linear.x = -linear_velocity_
    return msg

def main():
    global pub_, active_
    
    rospy.init_node('follow_wall')
    
    pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    
    sub = rospy.Subscriber('/scan', LaserScan, callback_laser)
    
    srv = rospy.Service('wall_follower_switch', SetBool, wall_follower_switch)
    
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        if not active_:
            rate.sleep()
            continue
        
        msg = Twist()
        if state_ == 0:
            msg = find_wall()
            print "Find wall Bitch!"
        elif state_ == 1:
            msg = turn_left()
            print "Turn Left Bitch!"
        elif state_ == 2:
            msg = follow_the_wall()
            print "Follow the Wall Bitch!"
            pass
        else:
            rospy.logerr('Unknown state! GG WP :(')
        
        pub_.publish(msg)
        
        rate.sleep()

if __name__ == '__main__':
    main()
    
