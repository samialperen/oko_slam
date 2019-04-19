#include "ros/ros.h"
#include "std_msgs/String.h"
#include <termios.h>
#include "kamu_robotu_comm/kamu_cmd.h"

#include <sstream>

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


int main(int argc, char **argv)
{  
  ros::init(argc, argv, "kamu_robotu_teleop");
 
  ros::NodeHandle n;

  ros::ServiceClient client = n.serviceClient<kamu_robotu_comm::kamu_cmd>("/kamu_cmd");
  kamu_robotu_comm::kamu_cmd srv;

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
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
             
        
    }
    
   
    ros::spinOnce();
    loop_rate.sleep();

  }


  return 0;
}
