#include "ros/ros.h"
#include "std_msgs/String.h"
#include <termios.h>
#include "kamu_robotu_comm/kamu_cmd.h"
#include <serial/serial.h>
#include <sstream>

serial::Serial ser;

int velocity = 30; 
int getch() //Function to take user input from the keyboard
{
  static struct termios oldt, newt;
  tcgetattr( STDIN_FILENO, &oldt);           // save old settings
  newt = oldt;
  newt.c_lflag &= ~(ICANON);                 // disable buffering      
  tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings

  int c = getchar();  // read character (non-blocking)

  tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
  return c;
}

bool kamu_command_handler(kamu_robotu_comm::kamu_cmd::Request &req, kamu_robotu_comm::kamu_cmd::Response &res)
{
uint8_t data2send[6];

data2send[0] = 0x55;
data2send[1] = (uint8_t)req.cmd_type;
data2send[2] = (uint8_t)req.cmd_param;
data2send[3] = (uint8_t)req.cmd_enable;
data2send[4] = 0;
data2send[5] = 0;
ser.write((const uint8_t *) data2send, 6);
res.result = true;
return true;
}

int main(int argc, char **argv)
{  
  ros::init(argc, argv, "kamu_robotu_teleop");
 
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("/kamu_cmd", kamu_command_handler);
  //ros::ServiceClient client = n.serviceClient<kamu_robotu_comm::kamu_cmd>("/kamu_cmd");
  //kamu_robotu_comm::kamu_cmd srv;

  try // Connect to the port
  {       
      ser.setPort("/dev/rfcomm1"); // miniuart port of the rpi, /dev/ttyS0
      ser.setBaudrate(9600);
      serial::stopbits_t  stopbits;
      stopbits = serial::stopbits_one;	
      ser.setStopbits(stopbits);
      serial::Timeout to = serial::Timeout::simpleTimeout(100);
      ser.setTimeout(to);
      ser.open();
  }
  catch (serial::IOException& e)
  {
      ROS_ERROR_STREAM("Unable to open porttttt ");
      return -1;
  }

  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    ROS_INFO("bambam");
    /*
    int keyboard_input = getch();
    switch (keyboard_input) {
        case 'w':
            srv.request.cmd_type = 1;
            srv.request.cmd_param = velocity;
            srv.request.cmd_enable = 1;
            client.call(srv);
            ROS_INFO("W");
            break;
	case 'a':
            srv.request.cmd_type = 3;
            srv.request.cmd_param = velocity;
            srv.request.cmd_enable = 1;
            client.call(srv);
            ROS_INFO("a");
            break;
	case 's':
            srv.request.cmd_type = 2;
            srv.request.cmd_param = velocity;
            srv.request.cmd_enable = 1;
            client.call(srv);
            ROS_INFO("s");
            break;
	case 'd':
            srv.request.cmd_type = 4;
            srv.request.cmd_param = velocity;
            srv.request.cmd_enable = 1;
            client.call(srv);
            ROS_INFO("d");
            break;
	case 'x':
            srv.request.cmd_type = 1;
            srv.request.cmd_param = velocity;
            srv.request.cmd_enable = 0;
            client.call(srv);
		
	    srv.request.cmd_type = 2;
            srv.request.cmd_param = velocity;
            srv.request.cmd_enable = 0;
            client.call(srv);

	    srv.request.cmd_type = 3;
            srv.request.cmd_param = velocity;
            srv.request.cmd_enable = 0;
            client.call(srv);

	    srv.request.cmd_type = 4;
            srv.request.cmd_param = velocity;
            srv.request.cmd_enable = 0;
            client.call(srv);

            ROS_INFO("x");
            break;
             
        
    }*/
    
   
    ros::spinOnce();
    loop_rate.sleep();

  }

  ser.close();
  return 0;
}
