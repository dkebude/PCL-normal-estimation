cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(estimate_normals_pcl_omp)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (estimate_normals_pcl_omp estimate_normals_pcl_omp.cpp)
target_link_libraries (estimate_normals_pcl_omp ${PCL_LIBRARIES})