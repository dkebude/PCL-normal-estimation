cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(estimate_normals_pcl_cuda)

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#Searching CUDA
FIND_PACKAGE(CUDA)

#Include the FindCUDA script
INCLUDE(FindCUDA)

cuda_add_executable (estimate_normals_pcl_cuda estimate_normals_pcl_cuda.cpp)
target_link_libraries (estimate_normals_pcl_cuda ${PCL_LIBRARIES})
