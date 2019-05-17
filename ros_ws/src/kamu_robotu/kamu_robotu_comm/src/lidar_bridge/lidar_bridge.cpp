/***
 * This example expects the serial port has a loopback on it.
 *
 *  Author : Mustafa KILINC
 *  E-mail : mustafa.kilinc@ieee.metu.edu.tr
 * Alternatively, you could use an Arduino:
 *
 * <pre>
 *  void setup() {
 *    Serial.begin(<insert your baudrate here>);
 *  }
 *
 *  void loop() {
 *    if (Serial.available()) {
 *      Serial.write(Serial.read());
 *    }
 *  }
 * </pre>
 */

#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <string>
#include <sensor_msgs/LaserScan.h>
#include "okoserial.h"



serial::Serial ser;
std::string eol="~~~~";
float w=0,x=0,y=0,z=0;
uint8_t dummy_8 = 0;
int16_t dummy_16 = 0;
float range[128];
int i = 0, f = 0;

std::string readable;

void write_callback(const std_msgs::String::ConstPtr& msg)
{
    	ROS_INFO_STREAM("Writing to serial port" << msg->data);
	ser.write(msg->data);
}

int main (int argc, char** argv){
    	ros::init(argc, argv, "laser_scan_publisher");
	ros::NodeHandle n;
	ros::Publisher scan_pub = n.advertise<sensor_msgs::LaserScan>("scan",50);

	unsigned int num_readings = 128;
	double laser_frequency = 1.2; // this true for 64 readings
	double ranges[num_readings];
	int count = 0 ;


    	try
    	{
       		ser.setPort("/dev/rfcomm0");
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
        	ser.flush();
    	}else{
        	return -1;
    	}

    	ros::Rate loop_rate(50);
	//populate the laser scan message

	sensor_msgs::LaserScan scan;

	scan.header.frame_id = "base_scan";
	scan.angle_min = 0;//-1.572;//2.3562;
        scan.angle_max = 6.2832;// -1.572;//8.6394;
	scan.angle_increment = 6.2832/num_readings;
	scan.time_increment = (1/laser_frequency)/(num_readings);
	scan.range_min = 0.092;
	scan.range_max = 1.20;
    	while(n.ok()){

        	ros::spinOnce();

        	if(ser.available())
		{
            		ROS_INFO_STREAM("Reading from serial port");
            		std_msgs::String result;
            		readable = ser.readline(620,eol);
			ROS_INFO("1-%d 2-%d ",readable[0], readable[263]);
			for(i=0;i<2*num_readings+8;i++)
			{
				dummy_8 = readable[i];
				getSingleByte(dummy_8);
				//ROS_INFO("DATA.%d : %x " , i , dummy_8);
				//ROS_INFO("laserscan_mess.%d :%d " , k, laserscan_mess.data[k]);
			}
				
		
			scan.ranges.resize(128);
			for(f = 0 ; f<128;f++)
			{

				scan.ranges[f] = laserscan_mess.data[f]/1000.0;

			}
			f=0;
			ros::Time scan_time = ros::Time::now();
			scan.header.stamp = scan_time;
			scan_pub.publish(scan);
	

			i=0;
			
        	}

		loop_rate.sleep();


    	}
        ser.close();
}



