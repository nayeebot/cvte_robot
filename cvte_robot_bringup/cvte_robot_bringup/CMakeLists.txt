cmake_minimum_required(VERSION 3.0.2)
project(cvte_robot_bringup)

catkin_package()
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})