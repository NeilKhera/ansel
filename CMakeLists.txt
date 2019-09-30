cmake_minimum_required(VERSION 2.8.3)
project(ansel)

find_package(catkin REQUIRED COMPONENTS rospy std_msgs sensor_msgs cv_bridge message_generation)
find_package(OpenCV REQUIRED)

add_service_files(FILES Ansel.srv)
generate_messages(DEPENDENCIES std_msgs sensor_msgs)

catkin_package(CATKIN_DEPENDS rospy std_msgs sensor_msgs cv_bridge)
