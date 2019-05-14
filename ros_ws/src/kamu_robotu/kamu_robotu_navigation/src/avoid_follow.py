#! /usr/bin/env python
# We are looking for the wall. If we find wall, we turn left to keep wall always
# right side of the robot

import random 
import rospy
import os
import time
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations
from std_srvs.srv import *

import math
import numpy as np

## Global variables
active_ = True
pub_ = None #Since we are using publisher in the functions as well we need to
# make it global

desired_duration_half = 0
current_duration = 0
dummy = 0

linear_velocity_ = 0.03
angular_velocity_ = 0.20

max_dist2robot = 0.25

regions = {
        'east':  0,
        'north': 0,
        'west':  0,
        'south': 0,
}
flags = {
        'east':  True,
        'north': True,
        'west':  True,
        'south': True,
}

state_ = 0 #Start from find the wall state
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
    global regions, flags
    global desired_duration_half
    global current_duration


    # Our laser publishes 128 point per rotation
    # Let's divide these 128 point to total_region_number different region
    # LIDAR rotates counter-clockwise
    # Let's start from the absolute right direction region and rotate cclockwise
    # By taking mean value in reach region, we want to find out which region
    # has the closest object/wall
    # To eliminate Inf measurement, we used mean function
    # Yet another alternative for eliminating ones smaller than 0.092 m = np.mean(filter(la$
    MAX_LIDAR_RANGE = 3 #in meter
    total_region_number = 6 #East, North, West, South
    msg.ranges = np.array(msg.ranges)
    msg.ranges[msg.ranges<0.093] = np.nan
#    east_array = msg.ranges[124:127] + msg.ranges[0:3]
    north_array1 = msg.ranges[121:127]
    north_array2 = msg.ranges[0:6]
#    north_east_west_west = np.nanmean(msg.ranges[119:120])
#    north_east_west = np.nanmean(msg.ranges[115:118])
    north_east = np.nanmean(msg.ranges[110:120])
#    north_east_east = np.nanmean(msg.ranges[106:109])
#    north_east_east_east = np.nanmean(msg.ranges[103:105])

 #   north_west_west_west = np.nanmean(msg.ranges[22:22])
#    north_west_west = np.nanmean(msg.ranges[19:22])
    north_west = np.nanmean(msg.ranges[7:18])
#    north_west_east = np.nanmean(msg.ranges[9:12])
#    north_west_east_east = np.nanmean(msg.ranges[7:8])


#    regions = {
#      'north': np.nanmean(msg.ranges[28:36]),
#      'west': np.nanmean(msg.ranges[60:68]),
#      'south': np.nanmean(msg.ranges[92:100]),
#      'east': np.nanmean(east_array),
#      'n-e': np.nanmean(msg.ranges[10:22]),
#      'n-w': np.nanmean(msg.ranges[42:54])
#     }
    regions = {
      'north': np.nanmin(np.concatenate((msg.ranges[121:127], msg.ranges[0:6]), axis=None)),
      'west':  np.nanmin(msg.ranges[19:32]),
      'south': np.nanmin(msg.ranges[60:68]),
      'east':  np.nanmin(msg.ranges[96:109]),
      'n-w':   np.nanmin(msg.ranges[7:18]), #(north_west_east_east, north_west_east, north_$
      'n-e':   np.nanmin(msg.ranges[110:120]) #(north_east_east_east, north_east_east, nort$
     }

    flags = {
        'north': regions['north'] < max_dist2robot,
        'west': regions['west'] < max_dist2robot,
        'south': regions['south'] < max_dist2robot,
        'east': regions['east'] < max_dist2robot,
        'n-w': regions['n-w'] < max_dist2robot,
        'n-e': regions['n-e'] < max_dist2robot,
     }
    if current_duration <= desired_duration_half:
        take_action_follow()
    else:
        take_action_avoid()

def change_state(state):
    global state_, state_dict_
    if state is not state_:
        print 'Wall follower - [%s] - %s' % (state, state_dict_[state])
        state_ = state
def take_action_follow():
    global linear_velocity_, angular_velocity_
    msg = Twist()
    linear_x = 0
    angular_z = 0
    state_description = ''

    # If there exists an object only north then turn left --> Case 2
    # Positive turn around z axis corresponds to turning left

    if flags['north'] and (not flags['west']) and (not flags['east']):
        state_description = 'case 2 - north'
        change_state(3)
    elif flags['n-w'] and (not flags['n-e']) and (not flags['north']):
        state_description = 'case 9 nw'
        change_state(3)
    elif (not flags['n-w']) and flags['n-e'] and (not flags['north']):
        state_description = 'case 10 ne'
        change_state(1)
    elif flags['n-w'] and flags['n-e'] and (not flags['north']):
        state_description = 'case 11 ne nw'
        change_state(0)
    elif flags['n-w'] and flags['n-e'] and flags['north']:
        state_description = 'case 11 ne nw n'
        change_state(3)
    else:
        if (not flags['north']) and (not flags['west']) and (not flags['east']):
            state_description = 'case 1 - nothing'
            change_state(0)
        elif flags['north'] and (not flags['west']) and (not flags['east']):
            state_description = 'case 2 - north'
            change_state(3)
        elif (not flags['north']) and (not flags['west']) and flags['east']:
            state_description = 'case 3 - east'
            change_state(2)
        elif (not flags['north']) and flags['west'] and (not flags['east']):
            state_description = 'case 4 - west'
            change_state(2)
        elif flags['north'] and (not flags['west']) and flags['east']:
            state_description = 'case 5 - north and east'
            change_state(1)
        elif flags['north'] and flags['west'] and (not flags['east']):
            state_description = 'case 6 - north and west'
            change_state(3)
        elif flags['north'] and flags['west'] and flags['east']:
            state_description = 'case 7 - north and west and east'
            change_state(3)
        elif (not flags['north']) and flags['west'] and flags['east']:
            state_description = 'case 8 - west and east'
            change_state(0)
        else:
            state_description = 'Unknown Case GG WP!'
            rospy.loginfo(regions)

    rospy.loginfo(state_description)
    rospy.loginfo(regions)


def take_action_avoid():
    global linear_velocity_, angular_velocity_, dummy
    msg = Twist()
    linear_x = 0
    angular_z = 0
    state_description = ''

    # If there exists an object only north then turn left --> Case 2
    # Positive turn around z axis corresponds to turning left

    if (not flags['n-w']) and (not flags['north']) and (not flags['n-e']) and not (flags['east']) and (not flags['west']) :
        state_description = 'case 2 - not n-w,n-e,north'
        change_state(0)
	dummy = bool(random.getrandbits(1))
    else:
	if(dummy == 0):
            state_description = 'case 3 - some  n-w,n-e,north-TURNRIGHT'
            change_state(3)
	else:
            state_description = 'case 1 - some  n-w,n-e,north-TURNLEFT'
            change_state(1)

    rospy.loginfo(state_description)
    rospy.loginfo(regions)



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
    msg = Twist()
    msg.linear.x = linear_velocity_
    msg.angular.z = 0
    return msg

def sd_hook():
    msg = Twist()
    msg.linear.x = 0
    msg.angular.z = 0
    pub_.publish(msg)

def main():
    global pub_, active_
    rospy.init_node('follow_wall')
    global desired_duration_half, current_duration

    pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    sub = rospy.Subscriber('/scan', LaserScan, callback_laser)

    srv = rospy.Service('wall_follower_switch', SetBool, wall_follower_switch)

    rospy.on_shutdown(sd_hook)

    current_duration = rospy.get_time()
    print "current_duration = " + str(current_duration)
    d = rospy.Duration.from_sec(60.0*13)
    desired_duration = current_duration + d.to_sec()
    desired_duration_half = current_duration + d.to_sec()/2
    print "desired_duration = " + str(desired_duration)
    rate = rospy.Rate(5)
    time.sleep(3);
    while not rospy.is_shutdown():
        if not active_:
            rate.sleep()
            continue
        msg = Twist()
        if state_ == 0:
            msg = find_wall()
            #print "Find wall"
        elif state_ == 1:
            msg = turn_left()
            #print "Turn Left"
        elif state_ == 3:
            msg = turn_right()
            #print "Turn Right Bitch"
        elif state_ == 2:
            msg = follow_the_wall()
            #print "Follow the Wall Bitch"
            pass
        else:
            rospy.logerr('Unknown state! GG WP :(')
        pub_.publish(msg)
        if current_duration >= desired_duration:
            try:
                os.system("rosnode kill /oko_bag")
                rospy.signal_shutdown('Good bye')
            except Exception as e:
                pass
        current_duration = rospy.get_time()
        rate.sleep()
if __name__ == '__main__':
    main()

