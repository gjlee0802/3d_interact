cmake_minimum_required(VERSION 3.8 FATAL_ERROR)

project(proj)

find_package(PCL 1.9 REQUIRED)

find_package(Boost COMPONENTS thread system REQUIRED)
find_package(Threads QUIET)

#find_package(CUDA)
#include(FindCUDA)


include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

SET(SRC_FILES src/main.cpp src/etc.cpp)

include_directories(./include)

add_executable (proj ${SRC_FILES})
target_link_libraries (proj ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})
