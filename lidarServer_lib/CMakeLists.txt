project(lidarServer_lib)

set(HEADER_FILES
        LidarServer.h
)

set(SOURCE_FILES
        LidarServer.cpp
        Point.h
)

add_library(lidarServer_lib STATIC ${SOURCE_FILES} ${HEADER_FILES})