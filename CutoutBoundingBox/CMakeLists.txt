cmake_minimum_required (VERSION 3.8)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")

set(PROJECT_BINARY_OUTPUT_PATH "${PROJECT_BINARY_DIR}")

set(QHULL_INC_DIR "/usr/local/include/libqhullcpp/")
set(QHULL_LIB_DIR "/usr/local/lib")
#set(QHULL_LIB "${QHULL_LIB_DIR}/libqhullcpp_d.a" "${QHULL_LIB_DIR}/libqhull_rd.so")
set(QHULL_LIB "${QHULL_LIB_DIR}/libqhullcpp.a" "${QHULL_LIB_DIR}/libqhull_r.so")

include_directories(${PCL_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/HeaderFiles)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(${QHULL_INC_DIR})
link_directories(${QHULL_LIB_DIR})

find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})

find_package(X11 REQUIRED)
include_directories(${X11_INCLUDE_DIR})

set (SOURCE_DIR "${PROJECT_SOURCE_DIR}/SourceFiles")
set (HEADER_DIR "${PROJECT_SOURCE_DIR}/HeaderFiles")

set (SOURCE_FILES "CutoutBoundingBox.cpp")
set (HEADER_FILES "CutoutBoundingBox.h" "${HEADER_DIR}/pclfuncs.h" "${HEADER_DIR}/BaseHeader.h" "${HEADER_DIR}/pcl-helpers.h" "${HEADER_DIR}/ndimage.h" "${HEADER_DIR}/narrowband.h" "${HEADER_DIR}/morphsnakes.h")

add_executable(CutoutBoundingBox "CutoutBoundingBox.cpp" "CutoutBoundingBox.h")
target_link_libraries(CutoutBoundingBox ${QHULL_LIB})
target_link_libraries(CutoutBoundingBox ${PCL_LIBRARIES})
target_link_libraries(CutoutBoundingBox ${realsense2_LIBRARY})
target_link_libraries(CutoutBoundingBox ${PYTHON_LIBRARIES})
target_link_libraries(CutoutBoundingBox ${X11_LIBRARIES})
