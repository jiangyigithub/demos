cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
set (CMAKE_CXX_STANDARD 11)

project(LIDAR_SEGMENTATION)
set(PCL_DIR,"/usr/lib/x86_64-linux-gnu/cmake/pcl/PCLConfig.cmake")
find_package(PCL 1.7 REQUIRED COMPONENTS common io)
find_package( OpenCV REQUIRED )

include_directories(${PCL_INCLUDE_DIRS} ../../include ./ lib/Eigen)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(kitti_parser  ../../src/gridsegmenter.cpp ../../include/gridsegmenter.h ../../include/GlobalHeader.h kitti_parser.cpp kitti_parser.h ../../include/blobtracker.h ../../src/blobtracker.cpp ../../include/HungarianMethod.h ../../src/HungarianMethod.cpp
    ../../include/convexhull.h ../../src/convexhull.cpp)
target_link_libraries(kitti_parser ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${OpenCV_LIBS})
