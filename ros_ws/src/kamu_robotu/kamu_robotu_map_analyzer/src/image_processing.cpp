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


int main(int argc, char** argv)
{
  ros::init(argc, argv, "map_analyzer");
  ros::NodeHandle nh;    
  
  std::string current_dir = GetCurrentWorkingDir();
  std::cout << current_dir << std::endl;
  
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
