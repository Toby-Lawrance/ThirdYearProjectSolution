cmake_minimum_required(VERSION 3.5)
project( object_tracking )

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

add_subdirectory(MainProject)
add_subdirectory(LibProject)