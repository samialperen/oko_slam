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



serial::Serial ser;
float w=0,x=0,y=0,z=0;
uint8_t dummy_8 = 0;
int16_t dummy_16 = 0;
float range[64];
int i = 0, f = 0;

std::string readable;

void write_callback(const std_msgs::String::ConstPtr& msg){
    ROS_INFO_STREAM("Writing to serial port" << msg->data);
    ser.write(msg->data);
}

int main (int argc, char** argv){
    ros::init(argc, argv, "laser_scan_publisher");
    //ros::NodeHandle nh;
	ros::NodeHandle n;


	ros::Publisher scan_pub = n.advertise<sensor_msgs::LaserScan>("scan",50);

    //ros::Subscriber write_sub = nh.subscribe("write", 1000, write_callback);
    //ros::Publisher read_pub = nh.advertise<std_msgs::String>("read", 1000);

	unsigned int num_readings = 64;
	double laser_frequency = 1; // this true for 64 readings
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
	scan.angle_min = 2.3562;
        scan.angle_max = 8.6394;
	scan.angle_increment = 6.2832/num_readings;
	scan.time_increment = (1/laser_frequency)/(num_readings);
	scan.range_min = 0.092;
	scan.range_max = 3.0;
    while(n.ok()){

        ros::spinOnce();

        if(ser.available()){
            ROS_INFO_STREAM("Reading from serial port");
            std_msgs::String result;
            readable = ser.read(1);


				if(readable[0]==119) {
					for(i=0;i<num_readings;i++){

						readable = ser.read(2);
						dummy_8 = readable[1];
						dummy_16 = (readable[0]<<8|dummy_8);
						ranges[i] = float(dummy_16/1.0);
						ROS_INFO_STREAM(ranges[i]);

					}
					scan.ranges.resize(num_readings);
	                for(f = 0 ; f<num_readings;f++){

		                scan.ranges[f] = ranges[f]/1000;

	                }
	                f=0;
					ros::Time scan_time = ros::Time::now();
					scan.header.stamp = scan_time;
					scan_pub.publish(scan);
				}

			i=0;
			//readable = ser.readline(1);
        	 //   ROS_INFO_STREAM(range);
            //read_pub.publish(readable[0]);
        }

		loop_rate.sleep();


    }
}


// IMU part

/*
	if(readable[0]==119) {
	readable = ser.readline(2);
	dummy_8 = readable[1];
	dummy_16 = (readable[0]<<8|dummy_8);
	w = float(dummy_16/10.0);
	ROS_INFO_STREAM(w);
	readable = ser.readline(1);
	}


	if(readable[0]==120) {
	readable = ser.readline(2);
	dummy_8 = readable[1];
	dummy_16 = (readable[0]<<8|dummy_8);
	x = float(dummy_16/10.0);
	ROS_INFO_STREAM(x);
	readable = ser.readline(1);
	}


	if(readable[0]==121) {
	readable = ser.readline(2);
	dummy_8 = readable[1];
	dummy_16 = (readable[0]<<8|dummy_8);
	y = float(dummy_16/10.0);
	ROS_INFO_STREAM(y);
	readable = ser.readline(1);
	}


	if(readable[0]==122) {
	readable = ser.readline(2);
	dummy_8 = readable[1];
	dummy_16 = (readable[0]<<8|dummy_8);
	z = float(dummy_16/10.0);
	ROS_INFO_STREAM(z);
	}

	*/

