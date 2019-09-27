cmake_minimum_required(VERSION 2.8.3)
project(rostopic2file)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs)
catkin_package(CATKIN_DEPENDS roscpp sensor_msgs)
include_directories(${catkin_INCLUDE_DIRS})

add_executable(rostopic2file src/rostopic2file.cpp)
target_link_libraries(rostopic2file ${catkin_LIBRARIES})
