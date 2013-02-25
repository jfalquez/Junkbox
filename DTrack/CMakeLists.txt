cmake_minimum_required( VERSION 2.8 )

project( DTrack )

find_package( Pangolin REQUIRED )
find_package( SceneGraph REQUIRED )
find_package( Kangaroo REQUIRED )
find_package( Sophus REQUIRED )
find_package( RPG REQUIRED )
find_package( MVL REQUIRED )
find_package( SensorFusion REQUIRED )

include_directories( ${Pangolin_INCLUDE_DIRS} )
include_directories( ${SceneGraph_INCLUDE_DIRS} )
include_directories( ${Kangaroo_INCLUDE_DIRS} )
include_directories( ${Sophus_INCLUDE_DIR} )
include_directories( ${RPG_INCLUDE_DIRS} )
include_directories( ${MVL_INCLUDE_DIRS} )
include_directories( ${SENSORFUSION_INCLUDE_DIRS} )

link_directories( ${Pangolin_LIBRARY_DIRS} )

IF(CMAKE_COMPILER_IS_GNUCXX)
 SET(CMAKE_CXX_FLAGS "-std=c++11 -g")
ENDIF()

list(APPEND HDRS
    Common.h
    GpuHelpers.h
    InitCamera.h
    Keyframes.h
    StreamHelpers.h
    GLPath.h
)

list(APPEND SRCS
    DTrack.cpp
)


add_executable( DTrack ${HDRS} ${SRCS} )

target_link_libraries( DTrack
                        ${Pangolin_LIBRARIES}
                        ${SceneGraph_LIBRARIES}
                        ${Kangaroo_LIBRARIES}
                        ${RPG_LIBRARIES}
                        ${MVL_LIBRARIES}
                        ${SENSORFUSION_LIBRARIES}
                     )
