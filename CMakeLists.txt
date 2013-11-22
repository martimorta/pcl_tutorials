cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(voxel_grid)

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (run_pcd_tutorials run_pcd_tutorials.cpp)
target_link_libraries (run_pcd_tutorials ${PCL_LIBRARIES})
