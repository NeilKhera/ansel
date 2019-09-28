/*#include <iostream>
#include <fstream>*/
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <rostopic2file/Rostopic2File.h>

using namespace std;

bool saveImage (rostopic2file::Rostopic2File::Request &req, rostopic2file::Rostopic2File::Response &res) {
  string rostopic = req.rostopic;
  string filepath = req.filepath;

  ros::Duration timeout(5.0);
  sensor_msgs::Image::ConstPtr msg = ros::topic::waitForMessage<sensor_msgs::Image>(rostopic, timeout);

  if (msg != NULL) {
    int sec = msg->header.stamp.sec;
    int nsec = msg->header.stamp.nsec;

    int image_width = msg->width;
    int image_height = msg->height;

    stringstream filename_stream;
    filename_stream << "img_" << sec << "_" << nsec << ".png";
    string filename = filename_stream.str();

  /*ofstream image_file((filepath + "/" + filename).c_str());
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
      image_file.close();*/

    cv::Mat image;
    try {
      image = cv_bridge::toCvShare(msg, msg->encoding)->image;
    } catch (cv_bridge::Exception) {
      res.info = "Writing image file failed!";
      return false;
    }

    cv::imwrite(filepath + filename, image);
    ROS_INFO("Saved %s", filename.c_str());
    return true;
    /*} else {
      res.info = "Writing image file failed!";
    }*/
  }
  res.info = "No messages received in 5 seconds. Timeout!";
  return false;
}

int main (int argc, char **argv) {
  ros::init(argc, argv, "rostopic2file");
  ros::NodeHandle nh;
  
  ros::ServiceServer image_service = nh.advertiseService("rostopic2file", saveImage);
  ros::spin();
  return 0;
}
