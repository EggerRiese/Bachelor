
cmake_minimum_required(VERSION 3.8)


find_package(Threads)

include_directories(${PROJECT_SOURCE_DIR}/include)
add_definitions(-std=c++11)

set (SOURCE_DIR "${PROJECT_SOURCE_DIR}/SourceFiles")
set (HEADER_DIR "${PROJECT_SOURCE_DIR}/HeaderFiles")

find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})

set (SOURCE_FILES "MorphSnakes.cpp")
set (HEADER_FILES "MorphSnakes.h" "${HEADER_DIR}/ndimage.h" "${HEADER_DIR}/narrowband.h" "${HEADER_DIR}/morphsnakes.h")

add_executable (MorphSnakes ${SOURCE_FILES} ${HEADER_FILES})
target_link_libraries(MorphSnakes ${CMAKE_THREAD_LIBS_INIT})
target_link_libraries(MorphSnakes ${PYTHON_LIBRARIES})
