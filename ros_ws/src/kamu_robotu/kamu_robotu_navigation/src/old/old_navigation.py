#! /usr/bin/env python

import rospy
import numpy as np
import math

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry  
from tf import transformations

## GLobal variables

pub = None #Since we are using publisher in the functions as well we need to 
# make it global
linear_velocity_ = 0.2
angular_velocity_ = 0.15

# Robot State Variables
position_ = Point()
yaw_ = 0
# State Machine for Go to Point
state_ = 0
# State 0 = Fix heading 
# State 1 = Go straight line
# State 2 = Done (We are at the desired position)

# Goal position
desired_position_ = Point()
desired_position_.x = 5
desired_position_.y = 13
desired_position_.z = 0 #2D --> Z coordinate is always zero!

# Precision parameters ()
yaw_precision_ = math.pi / 90  #2 degree tolerance
position_precision_ = 0.02 # 2cm


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

def callback_odom(msg):
    global position_
    global yaw_
    
    # position
    position_ = msg.pose.pose.position
    
    # yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    #roll_ = euler[0]
    #pitch_ = euler[1]
    yaw_ = euler[2]    
    
 
def take_action(regions):
    global linear_velocity_, angular_velocity_
    msg = Twist()
    linear_x = 0
    angular_z = 0
    max_dist2robot = 0.30 #If detected object is far away from this value, we 
    # don't consider it
    
    state_description = ''
    
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
        state_description = 'Undefined Case'
        rospy.loginfo(regions)

    rospy.loginfo(state_description)
    #rospy.loginfo(regions)
    msg.linear.x = -linear_x    
    msg.angular.z = -angular_z
    pub.publish(msg)    

def fix_yaw(des_pos):
    global yaw_, pub, yaw_precision_, state_, angular_velocity_
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = desired_yaw - yaw_
    
    twist_msg = Twist()
    if math.fabs(err_yaw) > yaw_precision_:
        twist_msg.angular.z = angular_velocity_ if err_yaw > 0 else -angular_velocity_
    
    pub.publish(twist_msg)
    
    # state change conditions
    if math.fabs(err_yaw) <= yaw_precision_:
        print 'Yaw error: [%s]' % err_yaw
        change_state(1)
        
def change_state(state):
    global state_
    state_ = state
    print 'State changed to [%s]' % state_

def go_straight_ahead(des_pos):
    global yaw_, pub, yaw_precision_, state_, position_precision_, linear_velocity_
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = desired_yaw - yaw_
    err_pos = math.sqrt(pow(des_pos.y - position_.y, 2) + pow(des_pos.x - position_.x, 2))
    
    if err_pos > position_precision_:
        twist_msg = Twist()
        twist_msg.linear.x = linear_velocity_
        pub.publish(twist_msg)
    else:
        print 'Position error: [%s]' % err_pos
        change_state(2)
    
    # state change conditions
    if math.fabs(err_yaw) > yaw_precision_:
        print 'Yaw error: [%s]' % err_yaw
        change_state(0)

def done():
    twist_msg = Twist()
    # Stop the robot
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0
    pub.publish(twist_msg)
 
def main():
    global pub
    
    rospy.init_node('wall_following_navigation') #Add namespace to launch file later

    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)    
    sub_laser = rospy.Subscriber("/scan", LaserScan, callback_laser)
    sub_odom = rospy.Subscriber("/odom", Odometry, callback_odom)

    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        if state_ == 0: #Fix heading state
            fix_yaw(desired_position_)
        elif state == 1: #Go straight line state
            go_straight_ahead(desired_position_)
        elif state == 2: #Done state i.e. we are at desired position
            done()
            pass
        else:
           rospy.logerr("Unknown State for Go to Point Algorithm!")
           pass
        rate.sleep()
    #rospy.spin()
if __name__ == '__main__':
    main()
    
    


