cmake_minimum_required(VERSION 2.8.3)
project(rostopic2file)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs message_generation)
add_service_files(FILES Rostopic2File.srv)
generate_messages(DEPENDENCIES std_msgs sensor_msgs)

catkin_package(CATKIN_DEPENDS std_msgs roscpp sensor_msgs)
include_directories(${catkin_INCLUDE_DIRS})

add_executable(rostopic2file src/rostopic2file.cpp)
target_link_libraries(rostopic2file ${catkin_LIBRARIES})
