cmake_minimum_required(VERSION 2.8.3)
project(velodyne)

# Prepend common/pjfa_cmake_modules/cmake/PJFACommon.cmake
find_package(pjfa_cmake_modules REQUIRED)
include(PJFACommon)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  vfc
  roscpp
  tf
  message_filters
  lidar_msgs
  velodyne_msgs
  driving_common
  nodelet
  geometry_msgs
  diagnostic_msgs
  diagnostic_updater
  roslaunch
  pcl_ros
  vlr_exception
  time_jitter_compensation
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS thread system)

# libxml used for Velodyne configuration
include($ENV{ROS_ROOT}/core/rosbuild/FindPkgConfig.cmake)
pkg_check_modules(LIBXML REQUIRED libxml++-2.6)
include_directories(${LIBXML_INCLUDE_DIRS})


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES velocore
  CATKIN_DEPENDS tf message_filters lidar_msgs velodyne_msgs driving_common nodelet geometry_msgs
    diagnostic_msgs diagnostic_updater pcl_ros vlr_exception time_jitter_compensation
#  DEPENDS z
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#Set the include paths
file (GLOB_RECURSE HEADERS
*.h
*.hpp
)

## Declare a cpp library
# add_library(velodyne
#   src/${PROJECT_NAME}/velodyne.cpp
# )

## Declare a cpp executable
# add_executable(velodyne_node src/velodyne_node.cpp)

add_library(velocore
  src/velodyne_driver_nodelet.cpp
  src/velodyneConfig.cpp
  src/velodyne_projector.cpp
  src/velodyne_projector_filter.cpp
  src/velodyne_projector_nodelet.cpp
  src/velodyne_point_cloud_nodelet.cpp
  src/velodyne_to_pointcloud_nodelet.cpp # NEW STUFF
  src/velodyne_raw_message_filter.cpp # NEW STUFF
  ${HEADERS}
)

add_dependencies(velocore ${catkin_EXPORTED_TARGETS})
target_link_libraries(velocore ${catkin_LIBRARIES})

add_executable(velodyne_read_params src/velodyne_read_params.cpp)
target_link_libraries(velodyne_read_params ${LIBXML_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES})

roslaunch_add_file_check(launch/velodyne.launch)
roslaunch_add_file_check(launch/velodyne_point_cloud_standalone.launch)
roslaunch_add_file_check(launch/velodyne_projector_standalone.launch)
roslaunch_add_file_check(launch/velodyne_to_pointcloud.launch) # NEW STUFF


## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(velodyne_node velodyne_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(velodyne_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

install_binary_targets(velocore velodyne_read_params)
install_shared_headers()
install_all_launch_files(Debug Release RelWithDebInfo MinSizeRel DevelRelease)
install_launch_files(BinaryRelease launch/velodyne.launch)
install(FILES nodelet_plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_velodyne.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
