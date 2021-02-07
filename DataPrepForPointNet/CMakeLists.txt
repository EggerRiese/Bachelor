cmake_minimum_required (VERSION 3.8)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")

set(QHULL_INC_DIR "/usr/local/include/libqhullcpp/")
set(QHULL_LIB_DIR "/usr/local/lib")
set(QHULL_LIB "${QHULL_LIB_DIR}/libqhullcpp_d.a" "${QHULL_LIB_DIR}/libqhull_rd.so")

set(PROJECT_BINARY_OUTPUT_PATH "${PROJECT_BINARY_DIR}")

include_directories(${PCL_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/HeaderFiles)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set (SOURCE_DIR "${PROJECT_SOURCE_DIR}/SourceFiles")
set (HEADER_DIR "${PROJECT_SOURCE_DIR}/HeaderFiles")

include_directories(${QHULL_INC_DIR})
link_directories(${QHULL_LIB_DIR})


set (SOURCE_FILES "DataPrepForPointNet.cpp")
set (HEADER_FILES "DataPrepForPointNet.h" "${HEADER_DIR}/pclfuncs.h" "${HEADER_DIR}/BaseHeader.h" "${HEADER_DIR}/pcl-helpers.h")

add_executable(DataPrepForPointNet "DataPrepForPointNet.cpp" "DataPrepForPointNet.h")
target_link_libraries(DataPrepForPointNet ${PCL_LIBRARIES})
target_link_libraries(DataPrepForPointNet ${realsense2_LIBRARY})
target_link_libraries(DataPrepForPointNet ${QHULL_LIB})