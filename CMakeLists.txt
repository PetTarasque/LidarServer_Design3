cmake_minimum_required(VERSION 3.26)
project(LidarServer)

set(CMAKE_CXX_STANDARD 17)

set(SOURCE_FILES main.cpp)
add_executable(lidarServer_run ${SOURCE_FILES})

include_directories(lidarServer_lib)
add_subdirectory(lidarServer_lib)

target_link_libraries(lidarServer_run lidarServer_lib)

add_subdirectory(Google_tests)



