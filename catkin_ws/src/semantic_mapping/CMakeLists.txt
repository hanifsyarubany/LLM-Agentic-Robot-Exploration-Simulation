cmake_minimum_required(VERSION 3.0.2)
project(semantic_mapping)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  tf
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(
  PROGRAMS
    scripts/semantic_mapper_node.py
    scripts/storing_coordinate.py
    scripts/semantic_map_visualizer.py
    scripts/historical_mapping_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
