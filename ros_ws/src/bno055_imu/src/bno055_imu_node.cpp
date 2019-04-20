/***
 * This ros node is used to publish sensor_msgs/imu messages from bno055 IMU to /bno055 topic 
 * 
 *  Author : Mustafa KILINC
 *  E-mail : mustafa.kilinc@ieee.metu.edu.tr
*/

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/Imu.h>
#include <string>
#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>
#include <unistd.h>
#include <string.h>
#include <getopt.h>
#include <time.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <tf/transform_datatypes.h>
#include "getbno055.h"

/* ------------------------------------------------------------ *
 * Global variables and defaults                                *
 * ------------------------------------------------------------ */


char opr_mode[9] = {0};
char pwr_mode[8] = {0};
char datatype[256];
char senaddr[256] = "0x28";

char calfile[256];
 

double linear_acceleration_covariance = 1.539e-5;
double angular_velocity_covariance = 7.615e-7;
double orientation_covariance = 2.89e-8;


/* ----------------------------------------------------------- *
 *  print_calstat() - Read and print calibration status        *
 * ----------------------------------------------------------- */
void print_calstat() {
   struct bnocal bnoc;
   /* -------------------------------------------------------- *
    *  Check the sensors calibration state                     *
    * -------------------------------------------------------- */
   int res = get_calstatus(&bnoc);
   if(res != 0) {
      printf("Error: Cannot read calibration state.\n");
      exit(-1);
   }

   /* -------------------------------------------------------- *
    *  Convert the status code into a status message           *
    * -------------------------------------------------------- */
    printf("Sensor System Calibration = ");
    switch(bnoc.scal_st) {
      case 0:
         printf("Uncalibrated\n");
         break;
      case 1:
         printf("Minimal Calibrated\n");
         break;
      case 2:
         printf("Mostly Calibrated\n");
         break;
      case 3:
         printf("Fully calibrated\n");
         break;
   }

   printf("    Gyroscope Calibration = ");
   switch(bnoc.gcal_st) {
      case 0:
         printf("Uncalibrated\n");
         break;
      case 1:
         printf("Minimal Calibrated\n");
         break;
      case 2:
         printf("Mostly Calibrated\n");
         break;
      case 3:
         printf("Fully calibrated\n");
         break;
   }

   printf("Accelerometer Calibration = ");
   switch(bnoc.acal_st) {
      case 0:
         printf("Uncalibrated\n");
         break;
      case 1:
         printf("Minimal Calibrated\n");
         break;
      case 2:
         printf("Mostly Calibrated\n");
         break;
      case 3:
         printf("Fully calibrated\n");
         break;
   }

   printf(" Magnetometer Calibration = ");
   switch(bnoc.mcal_st) {
      case 0:
         printf("Uncalibrated\n");
         break;
      case 1:
         printf("Minimal Calibrated\n");
         break;
      case 2:
         printf("Mostly Calibrated\n");
         break;
      case 3:
         printf("Fully calibrated\n");
         break;
   }
}



int main(int argc, char **argv)
{	
	// Start ROS node 
	ros::init(argc, argv, "bno055_imu_node");

	ros::NodeHandle n;
	ros::Publisher scan_pub = n.advertise<sensor_msgs::Imu>("bno055",50);
	ros::Rate loop_rate(100);

	struct bnogyr bnodg;
	struct bnoacc bnoda;
	struct bnoqua bnodqua;
      


	int res = -1;       // res = function retcode: 0=OK, -1 = Error

	opmode_t newmode;
	newmode = ndof;

        
	      

	/* ----------------------------------------------------------- *
	 *    open the I2C bus and connect to the sensor i2c address   *
	 * ----------------------------------------------------------- */
	get_i2cbus(senaddr);

	// Set IMU mode such that it can give us orientation quaternion
	res = set_mode(ndof);
	if(res != 0) {
		 printf("Error: could not set sensor mode %s [0x%02X].\n", opr_mode, newmode);
		 exit(-1);
      	}

	//Publish in loop
	while(n.ok()){
		
		sensor_msgs::Imu msg;
		msg.header.stamp = ros::Time::now();
		msg.header.frame_id = "imu"; // No frame 
		

		// Read gyroscope values 
      		res = get_gyr(&bnodg);

      		if(res != 0) {
	 		printf("Error: Cannot read gyroscope data.\n");
	 		exit(-1);
      		}

		
		msg.angular_velocity.x = bnodg.gdata_x ;
		msg.angular_velocity.y = bnodg.gdata_y ;
		msg.angular_velocity.z = bnodg.gdata_z ;
		
		// Read accelerometer values
		res = get_acc(&bnoda);
      		if(res != 0) {
         		printf("Error: Cannot read accelerometer data.\n");
        		 exit(-1);
      		}

		msg.linear_acceleration.x = bnoda.adata_x;
		msg.linear_acceleration.y = bnoda.adata_y;
		msg.linear_acceleration.z = bnoda.adata_z;

		res = get_qua(&bnodqua);
      		if(res != 0) {
         		printf("Error: Cannot read Quaternation data.\n");
        		exit(-1);
     		}

		msg.orientation.x = bnodqua.quater_x;
		msg.orientation.y = bnodqua.quater_y;
		msg.orientation.z = bnodqua.quater_z;
		msg.orientation.w = bnodqua.quater_w;

		msg.linear_acceleration_covariance[0] = linear_acceleration_covariance; 
		msg.linear_acceleration_covariance[4] = linear_acceleration_covariance; 
		msg.linear_acceleration_covariance[8] = linear_acceleration_covariance; 

		msg.angular_velocity_covariance[0] = angular_velocity_covariance;
		msg.angular_velocity_covariance[4] = angular_velocity_covariance;
		msg.angular_velocity_covariance[8] = angular_velocity_covariance;

		msg.orientation_covariance[0] = orientation_covariance;
		msg.orientation_covariance[4] = orientation_covariance;
		msg.orientation_covariance[8] = orientation_covariance;


		scan_pub.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();



	}

	return 0;
}
