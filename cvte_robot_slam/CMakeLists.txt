cmake_minimum_required(VERSION 3.0.2)
project(cvte_robot_slam)

catkin_package()
install(DIRECTORY config launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})