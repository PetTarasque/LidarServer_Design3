cmake_minimum_required(VERSION 3.16)
project(lidarServer_lib)

set(HEADER_FILES
        LidarServer.h
)

set(SOURCE_FILES
        LidarServer.cpp
        Point.h
)

add_library(lidarServer_lib STATIC ${SOURCE_FILES} ${HEADER_FILES})
target_link_libraries(lidarServer_lib ldlidar_driver)