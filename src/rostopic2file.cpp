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

    stringstream filename_stream;
    filename_stream << "img_" << sec << "_" << nsec << ".png";
    string filename = filename_stream.str();

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
