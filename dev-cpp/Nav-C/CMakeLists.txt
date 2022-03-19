cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(main)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
include_directories(include)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (main main.cpp src/dijkstra.cpp src/utils.cpp src/a_star.cpp src/d_star_lite.cpp src/jump_point_search.cpp src/lpa_star.cpp include/dijkstra.hpp include/planner.hpp include/utils.hpp include/a_star.hpp include/d_star_lite.hpp include/jump_point_search.hpp include/lpa_star.hpp)
target_link_libraries (main ${PCL_LIBRARIES})
