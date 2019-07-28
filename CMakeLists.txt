cmake_minimum_required(VERSION 3.8 FATAL_ERROR)

project(proj)

find_package(PCL 1.9 REQUIRED)

find_package(Boost COMPONENTS thread system REQUIRED)
find_package(Threads QUIET)

find_package(CUDA)
include(FindCUDA)


include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (proj depth.cpp)
target_link_libraries (proj ${PCL_LIBRARIES} ${Boost_LIBRARIES})
