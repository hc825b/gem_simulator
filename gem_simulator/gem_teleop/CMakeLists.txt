cmake_minimum_required(VERSION 3.0.2)
project(gem_teleop)

find_package(catkin REQUIRED COMPONENTS
  joy
  roscpp
  rospy
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES learning_joy
#  CATKIN_DEPENDS joy roscpp rospy
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(teleop_joy src/teleop_joy.cpp)
target_link_libraries(teleop_joy ${catkin_LIBRARIES})


