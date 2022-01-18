cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

# set(PCL_LIBRARY_DIRS "/usr/include/pcl-1.8/")
# add due to the error of "/usr/lib/gcc/x86_64-linux-gnu/7/../../../x86_64-linux-gnu/libSM.so: undefined reference to `uuid_generate@UUID_1.0'"
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -luuid")

add_definitions(-std=c++14)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

project(playback)

find_package(PCL 1.11 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")


add_executable (environment src/environment.cpp src/render/render.cpp src/processPointClouds.cpp)
target_link_libraries (environment ${PCL_LIBRARIES})
