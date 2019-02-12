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
    std::cout << current_dir << std::endl;

    // Saved map directory from slam package
    std::string read_dir = current_dir + "/saved_maps/" + map_name + ".pgm";
    std::cout << read_dir << std::endl;

    // Read saved map.pgm created by map_saver
    cv::Mat input_map_image, input_map_image_focused, input_map_image_gs;
    input_map_image = cv::imread(read_dir);
    // Crop taken gs image
    input_map_image_focused = input_map_image(cv::Rect(1024-150,1024-150,300,300));
    
    // Grayscale it
    cv::cvtColor( input_map_image_focused, input_map_image_gs, CV_BGR2GRAY );
    
    
    
    std::cout << "Image Channels: " << input_map_image_gs.channels() << std::endl;
    cv::namedWindow( "Display window", CV_WINDOW_NORMAL);
    cv::resizeWindow("Display window", 600, 800);
    cv::imshow("Display window",input_map_image_focused); //original map
    cv::waitKey(0);

    //Detect circles
    std::vector<cv::Vec3f> circles;
    /// Apply the Hough Transform to find the circles
    cv::HoughCircles(input_map_image_gs, circles, CV_HOUGH_GRADIENT, 1, input_map_image_gs.rows/1, 20, 10, 0, 0 );
    /// Draw the circles detected
    for( size_t i = 0; i < circles.size(); i++ )
    {
        cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int radius = cvRound(circles[i][2]);
        // circle center
        circle( input_map_image_focused, center, 1, cv::Scalar(0,255,0), -1, 8, 0 );
        // circle outline
        circle( input_map_image_focused, center, radius, cv::Scalar(0,0,255), 3, 8, 0 );
    }
 
    cv::namedWindow( "Display window2", CV_WINDOW_NORMAL);
    cv::resizeWindow("Display window2", 600, 800);
    cv::imshow("Display window2",input_map_image_focused);
    cv::waitKey(0);

//    // Find map boundaries
//    cv::Mat map_boundaries;
//    cv::Canny(input_map_image, map_boundaries, 30,90);
//    cv::namedWindow( "Map Boundaries", CV_WINDOW_NORMAL);
//    cv::resizeWindow("Map Boundaries", 600, 800);
//    cv::imshow("Map Boundaries",map_boundaries);
//    cv::waitKey(0);

    // Find map boundaries
    cv::Mat orig_map_im, orig_map_im_gs, orig_map_im_blur, boundaries;
    orig_map_im = cv::imread(read_dir);
    cv::cvtColor(orig_map_im, orig_map_im_gs, CV_BGR2GRAY);
    cv::GaussianBlur(orig_map_im_gs, orig_map_im_blur, cv::Size(3,3),0,0);
    cv::Canny(orig_map_im_blur, boundaries, 10,30);
    cv::namedWindow( "Map Boundaries", CV_WINDOW_NORMAL);
    cv::resizeWindow("Map Boundaries", 600, 800);
    cv::imshow("Map Boundaries",boundaries);
    cv::waitKey(0);

    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(boundaries, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
    //std::sort(contours.begin(), contours.end(), compareContourAreas);
    //std::vector<cv::Point> biggestContour = contours[contours.size()-1];
    int largest_area = 0, largest_contour_index = 0;
    cv::Rect bounding_rect;
    for( int i = 0; i< contours.size(); i++ ) // iterate through each contour.
    {
        double a = cv::contourArea( contours[i],false);  //  Find the area of contour
        if(a>largest_area){
            largest_area=a;
            largest_contour_index=i;                //Store the index of largest contour
            bounding_rect=cv::boundingRect(contours[i]); // Find the bounding rectangle for biggest contour
        }

    }
    cv::Mat dst(orig_map_im.rows,orig_map_im.cols,CV_8UC1,cv::Scalar::all(0));
    cv::Scalar color( 255,255,255);
    drawContours(dst , contours,largest_contour_index, color, CV_FILLED, 8, hierarchy ); // Draw the largest contour using previously stored index.
    rectangle(orig_map_im, bounding_rect,  cv::Scalar(0,255,0),1, 8,0);

    cv::namedWindow( "Rectangled", CV_WINDOW_NORMAL);
    cv::resizeWindow("Rectangled", 600, 800);
    cv::imshow( "Rectangled", orig_map_im );
    cv::waitKey(0);


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

// comparison function object
bool compareContourAreas ( std::vector<cv::Point> contour1, std::vector<cv::Point> contour2 ) {
    double i = fabs( contourArea(cv::Mat(contour1)) );
    double j = fabs( contourArea(cv::Mat(contour2)) );
    return ( i < j );
}
