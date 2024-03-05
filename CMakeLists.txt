cmake_minimum_required(VERSION 3.26)
project(LidarServer)

set(CMAKE_CXX_STANDARD 17)

add_executable(untitled1 main.cpp
        lidarServer_lib/LidarServer.cpp
        lidarServer_lib/LidarServer.h
        Google_tests/LidarServerTest.cpp)

include_directories(lidarServer_lib)
add_subdirectory(lidarServer_lib)

add_subdirectory(Google_tests)
