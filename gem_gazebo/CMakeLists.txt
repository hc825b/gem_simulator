cmake_minimum_required(VERSION 2.8.3)

project(gem_gazebo)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  rospy
  sensor_msgs
  geometry_msgs
  ackermann_msgs
  std_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/gem_control.py
  scripts/gem_sensor_info.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch media models worlds
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
