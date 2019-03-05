#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>
#include <string>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
double v,w;
void twistlistenerCallback(geometry_msgs::Twist cmd){
    v = cmd.linear.x;
    w = cmd.angular.z;
}
int main(int argc, char** argv){
  ros::init(argc, argv, "odometry_bridge");
  ros::NodeHandle n;
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  ros::Subscriber twist_sub = n.subscribe("/cmd_vel",100,twistlistenerCallback);
  tf::TransformBroadcaster odom_broadcaster;
  serial::Serial ser;
  
  std::string readable;
  unsigned int num_readings = 6;
  uint8_t dummy_8 = 0;
  int16_t dummy_16 = 0;
  int i = 0;
  
  double odometry_info[6]; // x,y,th,vx,vy,w
  
  try // Connect to the port
    {
        ser.setPort("/dev/ttyS0"); // miniuart port of the rpi
        ser.setBaudrate(115200);
		serial::stopbits_t  stopbits;
		stopbits = serial::stopbits_two;	
		ser.setStopbits(stopbits);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }

    if(ser.isOpen()){
        ROS_INFO_STREAM("Serial Port initialized");
    }else{
        return -1;
    }

  ros::Time current_time;

  ros::Rate r(100);
  while(n.ok()){

    ros::spinOnce();               // check for incoming messages
    current_time = ros::Time::now();
        if(ser.available()){
            ROS_INFO_STREAM("Reading from serial port");
            std_msgs::String result;
            readable = ser.readline(1);
	

				if(readable[0]==119) {
					for(i=0;i<num_readings;i++){

						readable = ser.readline(2);
						dummy_8 = readable[1];
						dummy_16 = (readable[0]<<8|dummy_8);
						odometry_info[i] = double(dummy_16/1.0);
						ROS_INFO_STREAM(odometry_info[i]);

					}
				}

			i=0;
        }
    // Send v and w to the robot here with serial
    // and here
    // and here
    
    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(odometry_info[2]);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = odometry_info[0];
    odom_trans.transform.translation.y = odometry_info[1];
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = odometry_info[0];
    odom.pose.pose.position.y = odometry_info[1];
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = odometry_info[3];
    odom.twist.twist.linear.y = odometry_info[4];
    odom.twist.twist.angular.z = odometry_info[5];

    //publish the message
    odom_pub.publish(odom);
    r.sleep();
  }
}
