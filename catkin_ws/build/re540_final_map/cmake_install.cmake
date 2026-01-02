# Install script for directory: /home/hanif/Hanif-Workspace/Homework_5/catkin_ws/src/re540_final_map

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/hanif/Hanif-Workspace/Homework_5/catkin_ws/build/re540_final_map/catkin_generated/installspace/re540_final_map.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/re540_final_map/cmake" TYPE FILE FILES
    "/home/hanif/Hanif-Workspace/Homework_5/catkin_ws/build/re540_final_map/catkin_generated/installspace/re540_final_mapConfig.cmake"
    "/home/hanif/Hanif-Workspace/Homework_5/catkin_ws/build/re540_final_map/catkin_generated/installspace/re540_final_mapConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/re540_final_map" TYPE FILE FILES "/home/hanif/Hanif-Workspace/Homework_5/catkin_ws/src/re540_final_map/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/re540_final_map" TYPE PROGRAM FILES "/home/hanif/Hanif-Workspace/Homework_5/catkin_ws/src/re540_final_map/scripts/fix_world_paths.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/re540_final_map/materials" TYPE DIRECTORY FILES "/home/hanif/Hanif-Workspace/Homework_5/catkin_ws/src/re540_final_map/materials/" REGEX "/\\.svn$" EXCLUDE)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/re540_final_map/worlds" TYPE DIRECTORY FILES "/home/hanif/Hanif-Workspace/Homework_5/catkin_ws/src/re540_final_map/worlds/" REGEX "/\\.svn$" EXCLUDE)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/re540_final_map/models" TYPE FILE FILES
    "/home/hanif/Hanif-Workspace/Homework_5/catkin_ws/src/re540_final_map/model.config"
    "/home/hanif/Hanif-Workspace/Homework_5/catkin_ws/src/re540_final_map/model.sdf"
    "/home/hanif/Hanif-Workspace/Homework_5/catkin_ws/src/re540_final_map/final_map.dae"
    "/home/hanif/Hanif-Workspace/Homework_5/catkin_ws/src/re540_final_map/Final_map.stl"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/re540_final_map/figure" TYPE DIRECTORY FILES "/home/hanif/Hanif-Workspace/Homework_5/catkin_ws/src/re540_final_map/figure/" REGEX "/\\.svn$" EXCLUDE)
endif()

