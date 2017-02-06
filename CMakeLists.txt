cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(greedy_projection)

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (greedy_projection src/greedy_projection.cpp)
target_link_libraries (greedy_projection ${PCL_LIBRARIES})

add_executable (statistical_removal src/statistical_removal.cpp)
target_link_libraries (statistical_removal ${PCL_LIBRARIES})

add_executable (pcl_visualizer src/pcl_visualizer.cpp)
target_link_libraries (pcl_visualizer ${PCL_LIBRARIES})

#add_executable (poly_sample src/poly_sample.cpp)
#target_link_libraries (poly_sample ${PCL_LIBRARIES})

add_executable (resampling src/resampling.cpp)
target_link_libraries (resampling ${PCL_LIBRARIES})

add_executable (concave_hull_2d src/concave_hull_2d.cpp)
target_link_libraries (concave_hull_2d ${PCL_LIBRARIES})

add_executable (remove_outliers src/remove_outliers.cpp)
target_link_libraries (remove_outliers ${PCL_LIBRARIES})

add_executable (region_growing_segmentation src/region_growing_segmentation.cpp)
target_link_libraries (region_growing_segmentation ${PCL_LIBRARIES})

add_executable (normal_estimation src/normal_estimation.cpp)
target_link_libraries (normal_estimation ${PCL_LIBRARIES})

add_executable (filters_for_modeling src/filters_for_modeling.cpp)
target_link_libraries (filters_for_modeling ${PCL_LIBRARIES})

#add_executable (bspline_fitting src/bspline_fitting.cpp)
#target_link_libraries (bspline_fitting ${PCL_LIBRARIES})
