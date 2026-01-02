cmake_minimum_required(VERSION 3.0.2)
project(orb_slam3_apriltag_fusion)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf
)

catkin_package()

catkin_install_python(PROGRAMS
  src/fusion_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
