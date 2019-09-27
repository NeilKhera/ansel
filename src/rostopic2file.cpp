#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

using namespace std;

string PATH_TO_FILES = "/home/pcams/Desktop/output/";

void imageCallback (const sensor_msgs::Image::ConstPtr& msg) {
  int sec = msg->header.stamp.sec;
  int nsec = msg->header.stamp.nsec;

  int image_width = msg->width;
  int image_height = msg->height;

  stringstream filename_stream;
  filename_stream << "img_" << sec << "_" << nsec << ".pgm";
  string filename = filename_stream.str();

  ofstream image_file(PATH_TO_FILES + filename);
  if (image_file.is_open()) {
    image_file << "P2\n";
    image_file << image_width << " " << image_height << "\n";
    image_file << "255\n";
    
    for (int i = 0; i < image_height; i++) {
      for (int j = 0; j < image_width; j++) {
        int val = msg->data[i * image_width + j];
	image_file << val << " ";
      }
      image_file << "\n";
    }
    image_file.close();
    ROS_ERROR("Saved %s", filename.c_str());
  } else {
    ROS_ERROR("ERROR: Writing image file failed!");
  }
  ros::shutdown();
}

int main (int argc, char **argv) {
  ros::init(argc, argv, "rostopic2file");
  ros::NodeHandle nh;
  
  ros::Subscriber image_sub = nh.subscribe("/camera/image_raw", 1, imageCallback);
  ros::spin();
  return 0;
}
