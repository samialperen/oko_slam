/* rosrun kamu_robotu_map_analyzer map_analyzer2 /
home/alperen/MY_WORKSPACES/oko_slam/ros_ws/saved_maps/real_results/cartographer_maps/4Objects128pt3 */

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

// Helper function to find a cosine of angle between vectors from pt0->pt1 and pt0->pt2
static double angle(cv::Point pt1, cv::Point pt2, cv::Point pt0);
// Helper function to display text in the center of a contour
void setLabel(cv::Mat& im, const std::string label, std::vector<cv::Point>& contour);


int main(int argc, char** argv)
{
    ros::init(argc, argv, "map_analyzer2");
    ros::NodeHandle nh("~");

    // Get parameter from user
    std::string file_dir = argv[1];
    std::string yaml_file = file_dir + ".yaml";
    std::string map_file = file_dir + ".pgm";
    
    // Read Original Map Image        
    cv::Mat input_map_image, input_map_image_gs;
    input_map_image = cv::imread(map_file);
    
    // Read yaml files
    YAML::Node config = YAML::LoadFile(yaml_file);
    float resolution = config["resolution"].as<float>();
    std::vector<float> origin_yaml = config["origin"].as<std::vector<float>>();
    int offset_coordinate_axis = input_map_image.size().height;
    int robot_initial_point_x = abs( origin_yaml[0]/resolution );
    int robot_initial_point_y = offset_coordinate_axis - abs( origin_yaml[1]/resolution );
    
    std::cout << "robot initial pose x: " << robot_initial_point_x << std::endl;
    std::cout << "robot initial pose y: " << robot_initial_point_y << std::endl;
        
    // Draw a Small Circle to Specify Initial Starting Point of Robot
    cv::circle( input_map_image, cv::Point(robot_initial_point_x,robot_initial_point_y), 1.0, cv::Scalar(0,0,255), -1, 8, 0 );
    // Draw Coordinate Axis whose origin is robot initial position
    cv::line( input_map_image, cv::Point(robot_initial_point_x,robot_initial_point_y) , cv::Point(robot_initial_point_x+25,robot_initial_point_y), cv::Scalar(0,0,255), 1,8); //X axis
    cv::line( input_map_image, cv::Point(robot_initial_point_x,robot_initial_point_y) , cv::Point(robot_initial_point_x,robot_initial_point_y-25), cv::Scalar(0,255,0), 1,8); //Y axis
    
//    cv::namedWindow( "Original Map", CV_WINDOW_NORMAL);    
//    cv::resizeWindow("Original Map", 600, 800);
//    cv::imshow("Original Map",input_map_image);
//    cv::waitKey(0);


    // Convert original map image to grayscale
    cv::cvtColor( input_map_image, input_map_image_gs, CV_BGR2GRAY );
    ////// Smooth the image to filter noise and find edges using Canny
    cv::Mat map_im_blurred, map_im_edges;
    cv::GaussianBlur(input_map_image_gs, map_im_blurred, cv::Size(3,3),0,0);
    cv::Canny(map_im_blurred, map_im_edges, 10,30);
//    cv::namedWindow( "Map Edges", CV_WINDOW_NORMAL);
//    cv::resizeWindow("Map Edges", 600, 800);
//    cv::imshow("Map Edges",map_im_edges);
//    cv::waitKey(0);
    
    ////// Find Contours of the map with largest area and get a rectangle around
    // findContours uses edges to detect contours which possible belong to
    // closed structures like objects
    std::vector<std::vector<cv::Point> > map_im_contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(map_im_edges, map_im_contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
    double largest_area = 0.0; 
    int largest_contour_index = 0; 
    double second_largest_area = 0.0; 
    int second_largest_contour_index = 0; 

    for( int i = 0; i< map_im_contours.size(); i++ )
    {
        double a = cv::contourArea( map_im_contours[i],false);
        //std::cout << "Area: " << a << std::endl;
        if(a>=largest_area && (a-largest_area) >= 500)
        {
            second_largest_area = largest_area;
            second_largest_contour_index = largest_contour_index;
            largest_area = a;
            largest_contour_index=i;
        }
        else if (a>second_largest_area && a<largest_area)
        {
            second_largest_area = a;
            second_largest_contour_index = i;
        }
//    // Debug          
//    cv::drawContours(input_map_image,map_im_contours,i
//    ,cv::Scalar(255,0,0),0,8,hierarchy);
//    cv::namedWindow( "Map Edges2", CV_WINDOW_NORMAL);
//    cv::resizeWindow("Map Edges2", 600, 800);
//    cv::imshow("Map Edges2",input_map_image);
//    cv::waitKey(0);
//    cv::drawContours(input_map_image,map_im_contours,i
//    ,cv::Scalar(0,0,0),0,8,hierarchy);                                          
    }
                  
    //convex hulls    
    std::vector<std::vector<cv::Point> >hull(map_im_contours.size());
    std::vector<std::vector<int> > hullsI(map_im_contours.size()); 
    std::vector<std::vector<cv::Vec4i>> defects(map_im_contours.size());
    for (int i = 0; i < map_im_contours.size(); i++)
    {
        cv::convexHull(map_im_contours[i], hull[i], false);
        cv::convexHull(map_im_contours[i], hullsI[i], false); 
        if(hullsI[i].size() > 3 )            
        {
            cv::convexityDefects(map_im_contours[i], hullsI[i], defects[i]);
        }
    }
    //REQUIRED contour is detected,then convex hell is found and also convexity defects are found and stored in defects

    if (largest_area>100){
        std::cout << "Area is found" << std::endl;
        cv::drawContours(input_map_image, hull, second_largest_contour_index, cv::Scalar(0, 255, 255), 2, 8, std::vector<cv::Vec4i>(), 0, cv::Point());

        /// Draw convexityDefects
        for(int j=0; j<defects[second_largest_contour_index].size(); ++j)
        {
            const cv::Vec4i& v = defects[second_largest_contour_index][j];
            float depth = v[3] / 256;
            if (depth > 10) //  filter defects by depth
            {
                int startidx = v[0]; 
                cv::Point ptStart(map_im_contours[second_largest_contour_index][startidx]);
                int endidx = v[1]; 
                cv::Point ptEnd(map_im_contours[second_largest_contour_index][endidx]);
                int faridx = v[2]; 
                cv::Point ptFar(map_im_contours[second_largest_contour_index][faridx]);

                cv::line(input_map_image, ptStart, ptEnd, cv::Scalar(0, 255, 0), 0.2);
                cv::line(input_map_image, ptStart, ptFar, cv::Scalar(0, 255, 0), 0.2);
                cv::line(input_map_image, ptEnd, ptFar, cv::Scalar(0, 255, 0), 0.2);
                cv::circle(input_map_image, ptFar, 1, cv::Scalar(0, 255, 0), 1);
            }
        }
    }
    
//    cv::namedWindow( "Convex Defects", CV_WINDOW_NORMAL);
//    cv::resizeWindow("Convex Defects", 600, 800);
//    cv::imshow( "Convex Defects", input_map_image );
//    cv::waitKey(0);


    cv::Rect bounding_rect=cv::boundingRect(map_im_contours[second_largest_contour_index]);
    cv::rectangle(input_map_image, bounding_rect,  cv::Scalar(255,0,0),1, 8,0);
    
    // Crop the original map image to eliminate redundant parts
    cv::Mat map_im_cropped;
    cv::Mat map_im_cropped_gs;
    map_im_cropped = input_map_image(bounding_rect);

//    cv::namedWindow( "Cropped Map Im", CV_WINDOW_NORMAL);
//    cv::resizeWindow("Cropped Map Im", 600, 800);
//    cv::imshow( "Cropped Map Im", map_im_cropped);
//    cv::waitKey(0);

    // Count edge numbers of contours to detect objects
    cv::cvtColor( map_im_cropped, map_im_cropped_gs, CV_BGR2GRAY );
    std::vector<std::vector<cv::Point> > cropped_contours;
    std::vector<cv::Vec4i> cropped_hierarchy;
    cv::findContours(map_im_cropped_gs, cropped_contours, cropped_hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

    for( int i = 0; i< cropped_contours.size(); i++ )
    {
        std::vector<cv::Point> approx_cont;
        cv::approxPolyDP(cropped_contours[i], approx_cont, 0.1*cv::arcLength(map_im_contours[i],true),true);
        std::cout << "Edge Number:" << approx_cont.size() << std::endl;
        
        if ( !cv::isContourConvex(approx_cont) )
        {
            continue;
        }
        else if (approx_cont.size() == 3 && cv::contourArea( cropped_contours[i],false) < largest_area/10)
        {
            setLabel(map_im_cropped, "TRI", cropped_contours[i]);    
        }
        if ( (approx_cont.size() >= 4 && approx_cont.size() <= 6) && cv::contourArea( cropped_contours[i],false) < (largest_area/100) ) 
        {
            int vtc = approx_cont.size();
            
            std::vector<double> cos;
            for (int j = 2; j < vtc+1; j++)
				cos.push_back(angle(approx_cont[j%vtc], approx_cont[j-2], approx_cont[j-1]));

			// Sort ascending the cosine values
			std::sort(cos.begin(), cos.end());

			// Get the lowest and the highest cosine
			double mincos = cos.front();
			double maxcos = cos.back();

			// Use the degrees obtained above and the number of vertices
			// to determine the shape of the contour
			if (vtc == 4 && mincos >= -0.1 && maxcos <= 0.3)
				setLabel(map_im_cropped, "RECT", cropped_contours[i]);
			else if (vtc == 5 && mincos >= -0.34 && maxcos <= -0.27)
				setLabel(map_im_cropped, "PENTA", cropped_contours[i]);
			else if (vtc == 6 && mincos >= -0.55 && maxcos <= -0.45)
				setLabel(map_im_cropped, "HEXA", cropped_contours[i]);
        }
        else if (approx_cont.size() >= 7 && cv::contourArea( cropped_contours[i],false) < (largest_area/50) )
        {
            // Detect and label circles
			double area = cv::contourArea(cropped_contours[i]);
			cv::Rect r = cv::boundingRect(cropped_contours[i]);
			int radius = r.width / 2;

			if (std::abs(1 - ((double)r.width / r.height)) <= 0.2 &&
			    std::abs(1 - (area / (CV_PI * std::pow(radius, 2)))) <= 0.2)
				setLabel(map_im_cropped, "CIR", cropped_contours[i]);        
        }
    }
    
   
    cv::namedWindow( "Detected Objects", CV_WINDOW_NORMAL);
    cv::resizeWindow("Detected Objects", 600, 800);
    cv::imshow("Detected Objects",map_im_cropped);
    cv::waitKey(0);



    //std::string output_image = file_dir + "_processed.png";
    //cv::imwrite(output_image, map_im_cropped);

    return 0;
}

/**
 * Helper function to find a cosine of angle between vectors
 * from pt0->pt1 and pt0->pt2
 */
static double angle(cv::Point pt1, cv::Point pt2, cv::Point pt0)
{
	double dx1 = pt1.x - pt0.x;
	double dy1 = pt1.y - pt0.y;
	double dx2 = pt2.x - pt0.x;
	double dy2 = pt2.y - pt0.y;
	return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}

/**
 * Helper function to display text in the center of a contour
 */
void setLabel(cv::Mat& im, const std::string label, std::vector<cv::Point>& contour)
{
	int fontface = cv::FONT_HERSHEY_SIMPLEX;
	double scale = 0.4;
	int thickness = 1;
	int baseline = 0;

	cv::Size text = cv::getTextSize(label, fontface, scale, thickness, &baseline);
	cv::Rect r = cv::boundingRect(contour);

	cv::Point pt(r.x + ((r.width - text.width) / 2), r.y + ((r.height + text.height) / 2));
	cv::rectangle(im, pt + cv::Point(0, baseline), pt + cv::Point(text.width, -text.height), CV_RGB(255,255,255), CV_FILLED);
	cv::putText(im, label, pt, fontface, scale, CV_RGB(0,0,0), thickness, 8);
}
