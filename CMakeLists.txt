cmake_minimum_required(VERSION 3.16)
project(LidarServer CXX)

set(CMAKE_CXX_STANDARD 17)

#Makebuildype? mettre en release

set(SOURCEFILES main.cpp)
add_executable(lidarServer_run ${SOURCEFILES})


include_directories(lidarServer_lib)
add_subdirectory(lidarServer_lib)

include_directories(ldlidar_sdk)
add_subdirectory(ldlidar_sdk)
#add_subdirectory(ldlidar_sdk/include)


target_link_libraries(lidarServer_run lidarServer_lib)
target_link_libraries(lidarServer_run ldlidar_driver)
target_link_libraries(lidarServer_run pthread)

add_subdirectory(Google_tests)

