cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(registration_closedform_project)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "../bin/")
find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (registration_closedform src/main.cpp)
target_link_libraries (registration_closedform ${PCL_LIBRARIES})
