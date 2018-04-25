cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(estimate_normals_cuda)

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} /mnt/kufs/scratch/dkebude16/Normal_Estimation_CUDA)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#Searching CUDA
FIND_PACKAGE(CUDA)

#Include the FindCUDA script
INCLUDE(FindCUDA)

get_directory_property(dir_defs DIRECTORY ${CMAKE_SOURCE_DIR} COMPILE_DEFINITIONS)
set(vtk_flags)
foreach(it ${dir_defs})
    if(it MATCHES "vtk*")
    list(APPEND vtk_flags ${it})
    endif()
endforeach()

foreach(d ${vtk_flags})
    remove_definitions(-D${d})
endforeach()

cuda_add_executable (estimate_normals_cuda estimate_normals_cuda.cpp estimate_normals_cuda.cu)
target_link_libraries (estimate_normals_cuda ${PCL_LIBRARIES})
