cmake_minimum_required(VERSION 2.8.3)

project(gem_gazebo)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS scripts/gem_control.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY config launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
