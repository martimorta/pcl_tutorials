cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(voxel_grid)

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (downsample downsample.cpp)
target_link_libraries (downsample ${PCL_LIBRARIES})

add_executable (outlier_rm stat_outlier_rm.cpp)
target_link_libraries (outlier_rm ${PCL_LIBRARIES})

add_executable (filter_by_fields filter_by_fields.cpp)
target_link_libraries (filter_by_fields ${PCL_LIBRARIES})

add_executable (cylinder_segmentation cylinder_segmentation.cpp)
target_link_libraries (cylinder_segmentation ${PCL_LIBRARIES})
