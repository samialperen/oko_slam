// ROS
#include <ros/ros.h>
#include <std_msgs/String.h>
// OpenCV
#include <opencv2/opencv.hpp>

// File system and regular C++ headers
#include <string>
#include <iomanip>
#include <sstream>

#include <boost/filesystem.hpp>
#include <dirent.h>
#include <sys/stat.h>
// GetCurrentDir
#include <unistd.h> 
#define GetCurrentDir getcwd

std::string GetCurrentWorkingDir(void);
bool compareContourAreas ( std::vector<cv::Point> contour1, std::vector<cv::Point> contour2 );


int main(int argc, char** argv)
{
    ros::init(argc, argv, "map_analyzer");
    ros::NodeHandle nh("~");
    std::string map_name;
    if (nh.getParam("map_name", map_name))
    {
        ROS_INFO("Got param: %s", map_name.c_str());
    }
    else
    {
        ROS_ERROR("Failed to get param 'map_name'");
    }

    std::string current_dir = GetCurrentWorkingDir();
    std::cout << "Current Directory: " << current_dir << std::endl;
    // Saved map directory from slam package
    std::string read_dir = current_dir + "/saved_maps/" + map_name + ".pgm";
    std::cout << "Reading Directory: " << read_dir << std::endl;

    ////// Read saved map.pgm created by ROS map_saver
    cv::Mat input_map_image, input_map_image_gs;
    input_map_image = cv::imread(read_dir);
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

//    // Find map boundaries
//    cv::Mat map_boundaries;
//    cv::Canny(input_map_image, map_boundaries, 30,90);
//    cv::namedWindow( "Map Boundaries", CV_WINDOW_NORMAL);
//    cv::resizeWindow("Map Boundaries", 600, 800);
//    cv::imshow("Map Boundaries",map_boundaries);
//    cv::waitKey(0);

//  struct dirent **depthfileNameList;
//  struct stat depthstat;
//  int nDepth;
//    
//  // Image read directory       
//  std::string obj_read_dir = read_dir;
//  obj_read_dir = obj_read_dir.append(obj_name + "/");  
//  std::string dummy_depth_dir = obj_read_dir;
//  std::string depth_dir = dummy_depth_dir.append("depth_desired_out2/"); //Read directory
//  std::cout << "Reading directory: " << depth_dir << std::endl;
//  
//  // Output image directory
//  std::string depth_out = obj_read_dir + "depth_thresholded_png/"; //Desired save directory
//  boost::filesystem::path depth_out_dir(depth_out);
//  boost::filesystem::create_directory(depth_out_dir);
//  std::cout << "Saving directory: " << depth_out << std::endl;             
//   
//  nDepth = scandir(depth_dir.c_str(),&depthfileNameList, NULL,alphasort);
//  std::cout << "The number of images found: " << nDepth-2 << std::endl;
//  if (nDepth<0)
//  { 
//      return 0;
//  } 
//  cv::namedWindow("Display window", CV_WINDOW_AUTOSIZE );
//  for (int counter = 1;counter<nDepth-1;counter++)
//  {
//      // Read images
//      std::stringstream ss;
//      ss << std::setw(6) << std::setfill('0') << counter; 
//      std::string filename = ss.str();
//      std::string depth_fullpath = depth_dir + filename + ".png";
//      std::cout << "Reading: " << depth_fullpath << std::endl;
//      cv::Mat cv_depth;
//      cv_depth = cv::imread(depth_fullpath, CV_LOAD_IMAGE_ANYCOLOR | CV_LOAD_IMAGE_ANYDEPTH);
//          
//      // Apply thresholding  (zero-inverted)
//      int threshold_level = 1300;
//      int max_value = 65535; //16 bit depth image!
//      int threshold_type = 4; // zero-inverted           
//      cv::Mat cv_depth_thresholded; 
//      cv::threshold(cv_depth,cv_depth_thresholded,threshold_level,max_value,threshold_type);
//      //cv::imwrite(depth_out + filename + ".png",  cv_depth_thresholded);  


//      // Display         
//      double min;
//      double max;
//      cv::minMaxIdx(cv_depth_thresholded, &min, &max);
//      cv::Mat adjMap;
//      // expand range to 0..255
//      cv_depth_thresholded.convertTo(adjMap,CV_8UC1, 255 / (max-min), -min); 
//      cv::imshow("Display window",adjMap);
//      cv::waitKey(0);  
//  } 

  return 0;
}


std::string GetCurrentWorkingDir( void ) {
  char buff[FILENAME_MAX];
  GetCurrentDir( buff, FILENAME_MAX );
  std::string current_working_dir(buff);
  return current_working_dir;
}

//// comparison function object
//bool compareContourAreas ( std::vector<cv::Point> contour1, std::vector<cv::Point> contour2 ) {
//    double i = fabs( contourArea(cv::Mat(contour1)) );
//    double j = fabs( contourArea(cv::Mat(contour2)) );
//    return ( i < j );
//}
//std::sort(contours.begin(), contours.end(), compareContourAreas);
//std::vector<cv::Point> biggestContour = contours[contours.size()-1];
