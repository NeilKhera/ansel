cmake_minimum_required(VERSION 2.8.3)
project(rostopic2file)

find_package(catkin REQUIRED COMPONENTS rospy std_msgs sensor_msgs cv_bridge message_generation)
find_package(OpenCV REQUIRED)

add_service_files(FILES Rostopic2File.srv)
generate_messages(DEPENDENCIES std_msgs sensor_msgs)

catkin_package(CATKIN_DEPENDS rospy std_msgs sensor_msgs cv_bridge)
#include_directories(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

#add_executable(rostopic2file src/rostopic2file.py)
#target_link_libraries(rostopic2file ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
