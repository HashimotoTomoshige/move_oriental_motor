cmake_minimum_required(VERSION 3.0.2)
project(motor_pub_node)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(motor_pub_node   src/motor_pub_node.cpp)

target_link_libraries(motor_pub_node ${catkin_LIBRARIES})