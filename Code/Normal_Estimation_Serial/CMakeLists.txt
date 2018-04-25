cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(estimate_normals_serial)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")

add_executable (estimate_normals_serial estimate_normals_serial.cpp)
target_link_libraries (estimate_normals_serial ${PCL_LIBRARIES})