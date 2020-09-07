cmake_minimum_required(VERSION 2.8.3)
project(gem_vision)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  rospy
  sensor_msgs
  geometry_msgs
  ackermann_msgs
  std_msgs
)

catkin_package(
)

install(PROGRAMS
  scripts/gem_vision.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.launch"
)
