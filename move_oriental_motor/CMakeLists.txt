cmake_minimum_required(VERSION 3.0.2)
project(move_oriental_motor)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  rospy
  std_msgs
)

catkin_package(
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(
  move_oriental_motor src/move_oriental_motor.cpp
)

target_link_libraries(
 move_oriental_motor
 ${catkin_LIBRARIES}
)

add_executable(
  read_oriental_motor src/read_oriental_motor.cpp
)

target_link_libraries(
 read_oriental_motor
 ${catkin_LIBRARIES}
)