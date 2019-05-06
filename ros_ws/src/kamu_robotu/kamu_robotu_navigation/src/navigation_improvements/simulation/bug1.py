#! /usr/bin/env python

import rospy
# import ros message
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf import transformations
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from std_srvs.srv import *

import math

srv_client_go_to_point_ = None
srv_client_wall_follower_ = None
yaw_ = 0
yaw_precision_ = 5 * (math.pi / 90)
position_ = Point()
initial_position_ = Point()
initial_position_.x = rospy.get_param('initial_x')
initial_position_.y = rospy.get_param('initial_y')
initial_position_.z = 0
desired_position_ = Point()
desired_position_.x = rospy.get_param('des_pos_x')
desired_position_.y = rospy.get_param('des_pos_y')
desired_position_.z = 0
regions_ = None
state_desc_ = ['Go to point', 'circumnavigate obstacle', 'go to closest point']
state_ = 0
circumnavigate_starting_point_ = Point()
circumnavigate_closest_point_ = Point()
count_state_time_ = 0 # seconds the robot is in a state
count_loop_ = 0
# 0 - go to point
# 1 - circumnavigate
# 2 - go to closest point

# callbacks
def clbk_odom(msg):
    global position_, yaw_
    
    # position
    position_ = msg.pose.pose.position
    
    # yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]

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

#    east_array = msg.ranges[124:127] + msg.ranges[0:3]
    north_array = msg.ranges[124:127] + msg.ranges[0:3]

    north_east_west = np.mean(msg.ranges[115:118])
    north_east = np.mean(msg.ranges[110:114])
    north_east_east = np.mean(msg.ranges[106:109])

    north_west_west = np.mean(msg.ranges[19:22])
    north_west = np.mean(msg.ranges[14:15])
    north_west_east = np.mean(msg.ranges[10:13])


#    regions_ = { 
#      'north': np.mean(msg.ranges[28:36]),
#      'west': np.mean(msg.ranges[60:68]),
#      'south': np.mean(msg.ranges[92:100]),
#      'east': np.mean(east_array),
#      'n-e': np.mean(msg.ranges[10:22]),
#      'n-w': np.mean(msg.ranges[42:54])
#     }
    regions_ = {
      'south': np.mean(msg.ranges[60:68]),
      'east':  np.mean(msg.ranges[97:102]),
      'west': np.mean(msg.ranges[23:38]),
      'north': np.mean(north_array),
      'n-e': min(north_east_east, north_east, north_east_west),
      'n-w': min(north_west_east, north_west, north_west_west),
     }


def change_state(state):
    global state_, state_desc_
    global srv_client_wall_follower_, srv_client_go_to_point_
    global count_state_time_
    count_state_time_ = 0
    state_ = state
    log = "state changed: %s" % state_desc_[state]
    rospy.loginfo(log)
    if state_ == 0:
        resp = srv_client_go_to_point_(True)
        resp = srv_client_wall_follower_(False)
    if state_ == 1:
        resp = srv_client_go_to_point_(False)
        resp = srv_client_wall_follower_(True)
    if state_ == 2:
        resp = srv_client_go_to_point_(False)
        resp = srv_client_wall_follower_(True)

def calc_dist_points(point1, point2):
    dist = math.sqrt((point1.y - point2.y)**2 + (point1.x - point2.x)**2)
    return dist

def normalize_angle(angle):
    if(math.fabs(angle) > math.pi):
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle

def main():
    global regions_, position_, desired_position_, state_, yaw_, yaw_error_allowed_
    global srv_client_go_to_point_, srv_client_wall_follower_
    global circumnavigate_closest_point_, circumnavigate_starting_point_
    global count_loop_, count_state_time_
    
    rospy.init_node('bug1')
    
    sub_laser = rospy.Subscriber('/scan', LaserScan, clbk_laser)
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
    
    rospy.wait_for_service('/go_to_point_switch')
    rospy.wait_for_service('/wall_follower_switch')
    rospy.wait_for_service('/gazebo/set_model_state')
    
    srv_client_go_to_point_ = rospy.ServiceProxy('/go_to_point_switch', SetBool)
    srv_client_wall_follower_ = rospy.ServiceProxy('/wall_follower_switch', SetBool)
    srv_client_set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    
    # set robot position
    model_state = ModelState()
    model_state.model_name = 'm2wr'
    model_state.pose.position.x = initial_position_.x
    model_state.pose.position.y = initial_position_.y
    resp = srv_client_set_model_state(model_state)
    
    # initialize going to the point
    change_state(0)
    
    rate_hz = 20
    rate = rospy.Rate(rate_hz)
    while not rospy.is_shutdown():
        if regions_ == None:
            continue
        
        if state_ == 0:
            if regions_['front'] > 0.15 and regions_['front'] < 1:
                circumnavigate_closest_point_ = position_
                circumnavigate_starting_point_ = position_
                change_state(1)
        
        elif state_ == 1:
            # if current position is closer to the goal than the previous closest_position, assign current position to closest_point
            if calc_dist_points(position_, desired_position_) < calc_dist_points(circumnavigate_closest_point_, desired_position_):
                circumnavigate_closest_point_ = position_
                
            # compare only after 5 seconds - need some time to get out of starting_point
            # if robot reaches (is close to) starting point
            if count_state_time_ > 5 and \
               calc_dist_points(position_, circumnavigate_starting_point_) < 0.2:
                change_state(2)
        
        elif state_ == 2:
            # if robot reaches (is close to) closest point
            if calc_dist_points(position_, circumnavigate_closest_point_) < 0.2:
                change_state(0)
                
        count_loop_ = count_loop_ + 1
        if count_loop_ == 20:
            count_state_time_ = count_state_time_ + 1
            count_loop_ = 0
            
        rate.sleep()

if __name__ == "__main__":
    main()

