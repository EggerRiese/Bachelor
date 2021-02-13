# CMakeList.txt : Top-level CMake project file, do global configuration
# and include sub-projects here.
#
cmake_minimum_required (VERSION 3.8)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")

project ("PCSegmentation")

#Intel Realsense
find_package(realsense2 REQUIRED)

#Point Cloud Library
find_package(PCL 1.10.1 REQUIRED)


 #Global Set
include_directories(${PCL_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/HeaderFiles)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(PROJECT_BINARY_OUTPUT_PATH "${PROJECT_BINARY_DIR}")

# Include sub-projects
add_subdirectory ("CutoutBoundingBox")
add_subdirectory ("MorphSnakes") 
add_subdirectory ("DataPrepForPointNet")