// rosrun kamu_robotu_map_analyzer map_analyzer /home/alperen/MY_WORKSPACES/oko_slam/ros_ws/saved_maps/simulation_results/carto-kamu_map_1.2m-maxrange

// ROS
#include <ros/ros.h>
#include <std_msgs/String.h>
// OpenCV
#include <opencv2/opencv.hpp>
//#include <opencv2/highgui/highgui.hpp>

// File system and regular C++ headers
#include <string>
#include <iomanip>
#include <sstream>
#include <vector>

#include <boost/filesystem.hpp>
#include <dirent.h>
#include <sys/stat.h>
// GetCurrentDir
#include <unistd.h> 

#include <yaml-cpp/yaml.h>


#define GetCurrentDir getcwd
// Function Prototypes
std::string GetCurrentWorkingDir(void);

int main(int argc, char** argv)
{
    ros::init(argc, argv, "map_analyzer");
    ros::NodeHandle nh("~");

    // Get parameter from user
    std::string file_dir = argv[1];
    std::cout << argv[0] << std::endl;
    std::cout << argv[1] << std::endl;
//    
//    if (nh.getParam("file_dir", file_dir))
//    {
//        ROS_INFO("Got param: %s", file_dir.c_str());
//    }
//    else
//    {
//        ROS_ERROR("Failed to get param 'file_dir'");
//    }

    //std::string current_dir = GetCurrentWorkingDir();
    //std::string yaml_file =  current_dir + "/saved_maps/simulation_results/" + map_name + ".yaml";
    std::string yaml_file = file_dir + ".yaml";
    std::cout << yaml_file << std::endl; 
    // Read yaml files
    YAML::Node config = YAML::LoadFile(yaml_file);
    float resolution = config["resolution"].as<float>();
    std::vector<float> origin_yaml = config["origin"].as<std::vector<float>>();
    int robot_initial_point_x = abs( origin_yaml[0]/resolution );
    int robot_initial_point_y = abs( origin_yaml[1]/resolution );
    // Debug
//    std::cout << "origin first element: " << origin_yaml[0] << std::endl;
//    std::cout << "origin second element: " << origin_yaml[1] << std::endl;
//    std::cout << "resolution: " << resolution << std::endl;
//    std::cout << "Robot Initial Point X: " << robot_initial_point_x << std::endl;
//    std::cout << "Robot Initial Point Y: " << robot_initial_point_y << std::endl;
    
    // Read Original Map Image 
    std::string map_file = file_dir + ".pgm";
       
    cv::Mat input_map_image, input_map_image_gs;
    input_map_image = cv::imread(map_file);
    // Draw a Small Circle to Specify Initial Starting Point of Robot
    cv::circle( input_map_image, cv::Point(robot_initial_point_x,robot_initial_point_y), 1.0, cv::Scalar(0,0,255), -1, 8, 0 );
    // Draw Coordinate Axis whose origin is robot initial position
    cv::line( input_map_image, cv::Point(robot_initial_point_x,robot_initial_point_y) , cv::Point(robot_initial_point_x+25,robot_initial_point_y), cv::Scalar(0,0,255), 1,8); //X axis
    cv::line( input_map_image, cv::Point(robot_initial_point_x,robot_initial_point_y) , cv::Point(robot_initial_point_x,robot_initial_point_y-25), cv::Scalar(0,255,0), 1,8); //Y axis
    
    cv::namedWindow( "Original Map", CV_WINDOW_NORMAL);    
    cv::resizeWindow("Original Map", 600, 800);
    cv::imshow("Original Map",input_map_image);
    cv::waitKey(0);


    // Convert original map image to grayscale
    cv::cvtColor( input_map_image, input_map_image_gs, CV_BGR2GRAY );
    ////// Smooth the image to filter noise and find edges using Canny
    cv::Mat map_im_blurred, map_im_edges;
    cv::GaussianBlur(input_map_image_gs, map_im_blurred, cv::Size(3,3),0,0);
    cv::Canny(map_im_blurred, map_im_edges, 10,30);
    cv::namedWindow( "Map Edges", CV_WINDOW_NORMAL);
    cv::resizeWindow("Map Edges", 600, 800);
    cv::imshow("Map Edges",map_im_edges);
    cv::waitKey(0);
    
    

    ////// Find Contours of the map with largest area and get a rectangle around
    // findContours uses edges to detect contours which possible belong to
    // closed structures like objects
    std::vector<std::vector<cv::Point> > map_im_contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(map_im_edges, map_im_contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
    int largest_area = 0, largest_contour_index = 0;
    cv::Rect bounding_rect; //Rectangle around largest contour
    for( int i = 0; i< map_im_contours.size(); i++ )
    {
        double a = cv::contourArea( map_im_contours[i],false);
        if(a>largest_area){
            largest_area=a;
            largest_contour_index=i;
            bounding_rect=cv::boundingRect(map_im_contours[i]);
        }
    }
    ////// Display Rectangle that contains largest contour area = outer map walls
    cv::rectangle(input_map_image, bounding_rect,  cv::Scalar(255,0,0),1, 8,0);
    cv::namedWindow( "Area that will be cropped", CV_WINDOW_NORMAL);
    cv::resizeWindow("Area that will be cropped", 600, 800);
    cv::imshow( "Area that will be cropped", input_map_image );
    cv::waitKey(0);

    ////// Crop the original map image to eliminate redundant parts
    cv::Mat map_im_cropped, map_im_cropped_gs;
    map_im_cropped = input_map_image(bounding_rect);
    cv::cvtColor( map_im_cropped, map_im_cropped_gs, CV_BGR2GRAY );
    cv::namedWindow( "Cropped Map Im", CV_WINDOW_NORMAL);
    cv::resizeWindow("Cropped Map Im", 600, 800);
    cv::imshow( "Cropped Map Im", map_im_cropped);
    cv::waitKey(0);



    //////// OBJECT DETECTION
    ////// CIRCLE DETECTION
    std::vector<cv::Vec3f> circles;
    cv::Mat im_cropped_gs_blurred;
    cv::GaussianBlur(map_im_cropped_gs, im_cropped_gs_blurred, cv::Size(9,9),2,2);
    cv::HoughCircles(im_cropped_gs_blurred, circles, CV_HOUGH_GRADIENT, 1, im_cropped_gs_blurred.rows/20, 250, 25, 0, 0 );
    /// Draw the circles detected
    for( size_t i = 0; i < circles.size(); i++ )
    {
        cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int radius = cvRound(circles[i][2]);
        // circle center
        cv::circle( map_im_cropped, center, 1, cv::Scalar(0,255,0), 1, 8, 0 );
        // circle outline
        cv::circle( map_im_cropped, center, radius, cv::Scalar(0,0,255), 1, 8, 0 );
    }

    cv::namedWindow( "Detected Circles", CV_WINDOW_NORMAL);
    cv::resizeWindow("Detected Circles", 600, 800);
    cv::imshow("Detected Circles",map_im_cropped);
    cv::waitKey(0);

    ////// TRIANGLE DETECTION
    // Dilate the image (bridge the gaps, objects grows)
    cv::Mat im_cropped_gs_dilated;
    int dilation_size = 3;
    cv::Mat element = cv::getStructuringElement(cv::MORPH_CROSS,
                                                cv::Size(2 * dilation_size + 1, 2 * dilation_size + 1),
                                                cv::Point(dilation_size, dilation_size) );
    cv::dilate(map_im_cropped_gs,im_cropped_gs_dilated,element);
    cv::namedWindow( "Dilated Image", CV_WINDOW_NORMAL);
    cv::resizeWindow("Dilated Image", 600, 800);
    cv::imshow("Dilated Image",im_cropped_gs_dilated);
    cv::waitKey(0);
    // Smooth the image
    cv::Mat im_crop_dil_blur;
    cv::GaussianBlur(im_cropped_gs_dilated, im_crop_dil_blur, cv::Size(5,5),0,0);
    cv::namedWindow( "Blurred Dilated Image", CV_WINDOW_NORMAL);
    cv::resizeWindow("Blurred Dilated Image", 600, 800);
    cv::imshow("Blurred Dilated Image",im_crop_dil_blur);
    cv::waitKey(0);

    // Adaptive threshold may be needed
    std::vector<std::vector<cv::Point> > cropped_contours;
    std::vector<cv::Vec4i> cropped_hierarchy;
    cv::findContours(map_im_cropped_gs, cropped_contours, cropped_hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
    std::cout << "Largest Area: " << largest_area << std::endl;
    for( int i = 0; i< cropped_contours.size(); i++ )
    {
        std::vector<cv::Point> approx_cont;
        cv::approxPolyDP(cropped_contours[i], approx_cont, 0.1*cv::arcLength(map_im_contours[i],true),true);
        std::cout << "Edge Number:" << approx_cont.size() << std::endl;
        if (approx_cont.size() == 3 && cv::contourArea( cropped_contours[i],false) < largest_area/4)
        {
            cv::drawContours(map_im_cropped,cropped_contours,i,cv::Scalar(0,255,255),0,8,cropped_hierarchy);
        }
        if (approx_cont.size() == 4 && cv::contourArea( cropped_contours[i],false) < (largest_area/100) )
        {
            cv::drawContours(map_im_cropped,cropped_contours,i,cv::Scalar(255,0,255),0,8,cropped_hierarchy);
        }
    }
    cv::namedWindow( "Detected Triangles + Circles + Rectangles", CV_WINDOW_NORMAL);
    cv::resizeWindow("Detected Triangles + Circles + Rectangles", 600, 800);
    cv::imshow("Detected Triangles + Circles + Rectangles",map_im_cropped);
    cv::waitKey(0);

    return 0;
}


std::string GetCurrentWorkingDir( void ) {
    char buff[FILENAME_MAX];
    GetCurrentDir( buff, FILENAME_MAX );
    std::string current_working_dir(buff);
    return current_working_dir;
}


