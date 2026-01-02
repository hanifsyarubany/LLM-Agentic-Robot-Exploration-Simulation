# Install script for directory: /home/hanif/Hanif-Workspace/Homework_5/catkin_ws/src/robot_controller

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/hanif/Hanif-Workspace/Homework_5/catkin_ws/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/hanif/Hanif-Workspace/Homework_5/catkin_ws/build/robot_controller/catkin_generated/installspace/robot_controller.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/robot_controller/cmake" TYPE FILE FILES
    "/home/hanif/Hanif-Workspace/Homework_5/catkin_ws/build/robot_controller/catkin_generated/installspace/robot_controllerConfig.cmake"
    "/home/hanif/Hanif-Workspace/Homework_5/catkin_ws/build/robot_controller/catkin_generated/installspace/robot_controllerConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/robot_controller" TYPE FILE FILES "/home/hanif/Hanif-Workspace/Homework_5/catkin_ws/src/robot_controller/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/robot_controller" TYPE PROGRAM FILES "/home/hanif/Hanif-Workspace/Homework_5/catkin_ws/build/robot_controller/catkin_generated/installspace/local_map_wall_avoider.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/robot_controller" TYPE PROGRAM FILES "/home/hanif/Hanif-Workspace/Homework_5/catkin_ws/build/robot_controller/catkin_generated/installspace/corridor_centering.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/robot_controller" TYPE PROGRAM FILES "/home/hanif/Hanif-Workspace/Homework_5/catkin_ws/build/robot_controller/catkin_generated/installspace/corridor_aligner_node.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/robot_controller" TYPE PROGRAM FILES "/home/hanif/Hanif-Workspace/Homework_5/catkin_ws/build/robot_controller/catkin_generated/installspace/apriltag_approaching.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/robot_controller" TYPE PROGRAM FILES "/home/hanif/Hanif-Workspace/Homework_5/catkin_ws/build/robot_controller/catkin_generated/installspace/main_controller.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/robot_controller" TYPE PROGRAM FILES "/home/hanif/Hanif-Workspace/Homework_5/catkin_ws/build/robot_controller/catkin_generated/installspace/main_controller_v2.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/robot_controller" TYPE PROGRAM FILES "/home/hanif/Hanif-Workspace/Homework_5/catkin_ws/build/robot_controller/catkin_generated/installspace/main_controller_v3.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/robot_controller" TYPE PROGRAM FILES "/home/hanif/Hanif-Workspace/Homework_5/catkin_ws/build/robot_controller/catkin_generated/installspace/main_controller_v4.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/robot_controller" TYPE PROGRAM FILES "/home/hanif/Hanif-Workspace/Homework_5/catkin_ws/build/robot_controller/catkin_generated/installspace/main_controller_v5.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/robot_controller" TYPE PROGRAM FILES "/home/hanif/Hanif-Workspace/Homework_5/catkin_ws/build/robot_controller/catkin_generated/installspace/entering_store_node.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/robot_controller" TYPE PROGRAM FILES "/home/hanif/Hanif-Workspace/Homework_5/catkin_ws/build/robot_controller/catkin_generated/installspace/pre_entering_node.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/robot_controller" TYPE PROGRAM FILES "/home/hanif/Hanif-Workspace/Homework_5/catkin_ws/build/robot_controller/catkin_generated/installspace/object_pickup_controller.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/robot_controller" TYPE PROGRAM FILES "/home/hanif/Hanif-Workspace/Homework_5/catkin_ws/build/robot_controller/catkin_generated/installspace/decision_makers.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/robot_controller" TYPE PROGRAM FILES "/home/hanif/Hanif-Workspace/Homework_5/catkin_ws/build/robot_controller/catkin_generated/installspace/object_approaching_grasping.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/robot_controller" TYPE PROGRAM FILES "/home/hanif/Hanif-Workspace/Homework_5/catkin_ws/build/robot_controller/catkin_generated/installspace/object_approaching_grasping_enhanced.py")
endif()

