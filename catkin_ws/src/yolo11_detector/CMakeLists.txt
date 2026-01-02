cmake_minimum_required(VERSION 3.0.2)
project(yolo11_detector)

## Find catkin and required components
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
)

## Declare the catkin package
catkin_package()

## Install Python scripts
catkin_install_python(
  PROGRAMS
    scripts/yolo11_node.py
    scripts/yolo11_3D_ros_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install models and launch files to share directory (optional but nice)
install(
  DIRECTORY launch models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
