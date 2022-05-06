cmake_minimum_required(VERSION 3.5)
project(track_to_track_fusion)

## Compile as C++14, supported in ROS Kinetic and newer
set(CMAKE_CXX_STANDARD 14)

## Compile with warnings as errors
if( "${CMAKE_COMPILER_IS_GNUCXX}" OR "${CMAKE_CXX_COMPILER_ID}" MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(DEFINED ENV{ROS_VERSION})
    if("$ENV{ROS_VERSION}" STREQUAL "1")
        message (NOTICE "ROS1 is sourced. Configuring project for ROS1 ...")
        include(${CMAKE_CURRENT_SOURCE_DIR}/CMakeListsRos1.txt)
    elseif("$ENV{ROS_VERSION}" STREQUAL "2")
        message (NOTICE "ROS2 is sourced. Configuring project for ROS2 ...")
        include(${CMAKE_CURRENT_SOURCE_DIR}/CMakeListsRos2.txt)
    else()
        message(SEND_ERROR "Unknown ROS version! (Should be '1' or '2'). Won't continue with configuration!")
    endif()
else()
    message(SEND_ERROR "Environment variable 'ROS_VERSION' is undefined! Won't continue with configuration!")
endif()
